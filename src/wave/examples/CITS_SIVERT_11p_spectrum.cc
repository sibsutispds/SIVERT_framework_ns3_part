/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
  This software was developed at the National Institute of Standards and
  Technology by employees of the Federal Government in the course of
  their official duties. Pursuant to titleElement 17 Section 105 of the United
  States Code this software is not subject to copyright protection and
  is in the public domain.
  NIST assumes no responsibility whatsoever for its use by other parties,
  and makes no guarantees, expressed or implied, about its quality,
  reliability, or any other characteristic.

  We would appreciate acknowledgement if the software is used.

  NIST ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS" CONDITION AND
  DISCLAIM ANY LIABILITY OF ANY KIND FOR ANY DAMAGES WHATSOEVER RESULTING
  FROM THE USE OF THIS SOFTWARE.

 * Modified by: Nikita Lyamin <nikita.lyamin@volvocars.com> (VCC)
 *              Aeksei Fedorov <aleksei.fedorov@eit.lth.se> (LTH)
 */


#include "ns3/core-module.h"
#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/mobility-model.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/animation-interface.h"

// WAVE includes
#include "ns3/itu-r-1411-los-propagation-loss-model.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/config-store-module.h"
#include "ns3/integer.h"
#include "ns3/wave-bsm-helper.h"
#include "ns3/bsm-application.h"
#include "ns3/wave-helper.h"
#include <ns3/friis-spectrum-propagation-loss.h>
#include "ns3/multi-model-spectrum-channel.h"
#include <ns3/spectrum-model.h>
#include "ns3/spectrum-wifi-helper.h"
#include <ns3/sivert-spectrum-propagation-loss.h>
#include <ns3/sivert-model-spectrum-channel.h>

// Includes for SIVERT API
#include <zhelpers.hpp> // For ease we use zhelpers instead of zmq.hpp
#include <zmq.hpp>
#include "../flatbuffer_structures/positionSchema_generated.h" // include for our API's flatbuffer structure
#include "../flatbuffer_structures/MsgReceivedSchema_generated.h" // include for our API's flatbuffer structure

// General includes
#include <iostream>


using namespace ns3;
using namespace SivertAPI::PosUpd; // namespace for Position API's flatbuffers structures
using namespace SivertAPI::MsgReceived; // namespace for Msg API's flatbuffers structures

NS_LOG_COMPONENT_DEFINE ("SivertGSCM");


zmq::context_t context (1); // NB: there should be only one context in one thread --> thus, SINGLE context for entire NS3 simulation.
// Global variables for Pub and Sub ZeroMQ API connections
zmq::socket_t* APIconnect_sub;
zmq::socket_t* APIconnect_pub;

// Super IMPORTANT! We should use only one ZMQ context for our API!
static zmq::socket_t * s_sub_socket (zmq::context_t & context, std::string address) {

/*
 * For our needs one ZeroMQ context is enough (as a rule of thumb 1 context is required per 1 Gbit/s data exchange).
 * However, one could create numerous sockets using different ZeroMQ patterns for the API needs. Currently we use
 * Cleint/Server socket and PUB/SUB socket for our API.
*/

    zmq::socket_t * sck = new zmq::socket_t (context, ZMQ_SUB);
    sck->setsockopt( ZMQ_LINGER, 0 );             // ALWAYS, best before .bind()/.connect()
//    sck->setsockopt(ZMQ_RCVBUF, 1000000);
    sck->bind(address);
    sck->setsockopt( ZMQ_SUBSCRIBE, "toNS", 4);
    std::cout << "API SUB socket is created..." << std::endl;

    return sck;
}

// Super IMPORTANT! We should use only one ZMQ context for our API!
static zmq::socket_t * s_pub_socket (zmq::context_t & context, std::string address) {

/*
 * For our needs one ZeroMQ context is enough (as a rule of thumb 1 context is required per 1 Gbit/s data exchange).
 * However, one could create numerous sockets using different ZeroMQ patterns for the API needs. Currently we use
 * Cleint/Server socket and PUB/SUB socket for our API.
*/

    zmq::socket_t * sck = new zmq::socket_t (context, ZMQ_PUB);
    sck->setsockopt( ZMQ_LINGER, 0 );             // ALWAYS, best before .bind()/.connect()
//    sck->setsockopt(ZMQ_RCVBUF, 1000000);
    sck->bind(address);
//    sck->setsockopt( ZMQ_, "fromNS", 6);
    std::cout << "API PUB socket is created..." << std::endl;

    return sck;
}

void SetAllPositionFromAPI (NodeContainer Container, zmq::socket_t * APIconnect, double FixedUpdInterval)
{
/*
 * APIconnect - pointer to ZeroMQ socket SUB socket created in the main
 * FixedUpdInterval - Unity3D FixedUpdInterval we get from intiSocket from Unity.
 * 					  FixedUpd defines the physics updates in Unity and we want to synch NS3
 * 					  via our API  every time new values are calculated in Unity: for this we
 * 					  call PUB API each void FixedUpd and call the API SUB on NS3 side.
 *
*/


    std::cout<< "Receiving update from Unity3D"<<std::endl;

    std::string topic = s_recv (*APIconnect); // receive TOPIC
    std::string msg = s_recv (*APIconnect); // receive API message

    // Unwrapping the Flatbuffer API data structure we received from Unity
    flatbuffers::FlatBufferBuilder fbb;
    PosAPIBuilder builder(fbb);
    auto APIdata = GetPosAPI(msg.data());
    auto info = APIdata->Suppl();
    auto posit = APIdata->pos();
    bool termin = APIdata->terminateNS3();
//  auto EmergencyBrake = APIdata->CITS();
    auto EmergencyBrake = APIdata->CITS();


    for (unsigned int i = 0; i< EmergencyBrake->size(); i++){

        if (EmergencyBrake->Get(i)->triggered()) {

            try {
                Ptr<Node> veh = Container.Get (EmergencyBrake->Get(i)->VehID());
                Ptr<BsmApplication> app = veh->GetApplication(0)->GetObject<BsmApplication>();
                app->EEBL = true;
                app->EEBLtriggerVehID = EmergencyBrake->Get(i)->VehID();
            } catch (const std::exception& e) {
                std::cout<< "ERROR:" << e.what() << std::endl;
            }

        }
    }


    for (unsigned int i = 0; i < posit->size() ; i++) {


//      int veh = posit->size() - i - 1;
//      int veh = info->Get(i)->id();
        int veh = posit->Get(i)->id();
        Ptr<ConstantPositionMobilityModel> mobil = Container.Get(veh)->GetObject<ConstantPositionMobilityModel>();
        mobil->SetPosition(Vector(posit->Get(i)->x(), posit->Get(i)->z(), posit->Get(i)->y()));
//      std::cout<< "Position update received from Unity3D for Veh: " << veh << " With NS3 ID: "<< Container.Get(veh)->GetId() <<  " Position" << " x: " <<  posit->Get(i)->x()<< " y: " << posit->Get(i)->y()<< " z: " << posit->Get(i)->z()<< " EEB is set to: "<< EmergencyBrake->Get(EmergencyBrake->Get(i)->VehID())->triggered() << std::endl;
        std::cout<< "Position update received from Unity3D for Veh: " << veh << " With NS3 ID: "<< Container.Get(veh)->GetId() <<  " Position" << " x: " <<  posit->Get(i)->x()<< " y: " << posit->Get(i)->y()<< " z: " << posit->Get(i)->z()<< std::endl;
    }

    std::cout<< "Unity3D time received: " << info->Get(0)->timeStamp() << " NS3 current time: "<< Simulator::Now() << std::endl;

    // If Unity3D simulation is about to stop and we recieved termination message
    if(termin){
        std::cout<< "Command to terminate received from Unity3D. Stopping simulation and closing NS3 thread now!" << std::endl;
        APIconnect_pub->close();
        APIconnect_sub->close();
        Simulator::Stop();
    }


    Simulator::Schedule (Seconds(FixedUpdInterval), &SetAllPositionFromAPI, Container, APIconnect, FixedUpdInterval);

}

void SetAllPositionFromAPI (NodeContainer Container, zmq::socket_t * APIconnect, double FixedUpdInterval, const Ptr<SivertSpectrumPropagationLossModel> SpectrumModelPtr)
{
/*
 * APIconnect - pointer to ZeroMQ socket SUB socket created in the main
 * FixedUpdInterval - Unity3D FixedUpdInterval we get from intiSocket from Unity.
 * 					  FixedUpd defines the physics updates in Unity and we want to synch NS3
 * 					  via our API  every time new values are calculated in Unity: for this we
 * 					  call PUB API each void FixedUpd and call the API SUB on NS3 side.
 *
*/


    std::cout<< "Receiving update from Unity3D"<<std::endl;

    std::string topic = s_recv (*APIconnect); // receive TOPIC
    std::string msg = s_recv (*APIconnect); // receive API message

    // Unwrapping the Flatbuffer API data structure we received from Unity
    flatbuffers::FlatBufferBuilder fbb;
    PosAPIBuilder builder(fbb);
    auto APIdata = GetPosAPI(msg.data());
    auto info = APIdata->Suppl();
    auto posit = APIdata->pos();
    bool termin = APIdata->terminateNS3();
//  auto EmergencyBrake = APIdata->CITS();
    auto EmergencyBrake = APIdata->CITS();
    auto GSCMunity = APIdata->GSCM();
    auto GSCMvec = APIdata->GSCMvector();


  auto SpectumChVector = APIdata->GSCMSpectruChannels();
  auto SpectrumChInfo = APIdata->SpectrumInfo();
  auto NumFr = APIdata->NumberOfFrequnciesPerChannel();

  std::cout << " Size of spectrum channel info: " << SpectrumChInfo->size() << std::endl;
  for (int ch = 0; ch < SpectrumChInfo->size() ; ++ch)
    {

//      std::cout << " Current channel index: " << ch << std::endl;
      int TxID = SpectrumChInfo->Get(ch)->TxID();
      int RxID = SpectrumChInfo->Get(ch)->RxID();
      int ChID = SpectrumChInfo->Get(ch)->ChannelIDinfo();
      std::cout<< "Spectrum ChID: " << ChID << " TxID: " << TxID << " RxID: " << RxID << std::endl;

      std::vector<double> gscmSpectrumGain;
      for (int fr = NumFr-1; fr > -1; --fr)
        {
          int indFr = ch*(NumFr) + fr;
          double SpecGain = SpectumChVector->Get(indFr)->PSDGainCoefficient();
          int ChIDspecVec = SpectumChVector->Get(indFr)->ChannelID();
          int IndFrSpecCh = SpectumChVector->Get(indFr)->CoefInd();
//          std::cout<< "SpecGain: " << SpecGain << " ChIDspecVec: " << ChIDspecVec << " IndFrSpecCh: " << IndFrSpecCh << std::endl;
          gscmSpectrumGain.push_back(SpecGain);
        }
      SpectrumModelPtr->SetChannelFromUnity3D(TxID,RxID,gscmSpectrumGain);

    }


//    GSCMunity->UseGSCM()
    if (GSCMunity->UseGSCM()){
      std::cout<< "GSCM bool: " << GSCMunity->UseGSCM() << " GSCM size: " << GSCMvec->size()<< std::endl;
        for(unsigned int l =0; l< GSCMvec->size(); l++){

            int ind = GSCMvec->size() - l - 1;
//          int ind = l;
//          std::cout<< "GSCM size " << GSCMvec->size()<< " index is: " << ind << std::endl;
            std::cout<< "average RSS per 1MHz estimation GSCM for Tx: "<< GSCMvec->Get(ind)->Tx()<< " Rx: "<< GSCMvec->Get(ind)->Rx() << " from Unity to NS3 with: "<< GSCMvec->Get(ind)->rss() << " dBm/MHz" << std::endl;

        }
    }

    for (unsigned int i = 0; i< EmergencyBrake->size(); i++){

        if (EmergencyBrake->Get(i)->triggered()) {

            try {
                Ptr<Node> veh = Container.Get (EmergencyBrake->Get(i)->VehID());
                Ptr<BsmApplication> app = veh->GetApplication(0)->GetObject<BsmApplication>();
                app->EEBL = true;
                app->EEBLtriggerVehID = EmergencyBrake->Get(i)->VehID();
            } catch (const std::exception& e) {
                std::cout<< "ERROR:" << e.what() << std::endl;
            }

        }
    }


    for (unsigned int i = 0; i < posit->size() ; i++) {


        int veh = posit->Get(i)->id();
        Ptr<ConstantPositionMobilityModel> mobil = Container.Get(veh)->GetObject<ConstantPositionMobilityModel>();
        mobil->SetPosition(Vector(posit->Get(i)->x(), posit->Get(i)->z(), posit->Get(i)->y()));
//      std::cout<< "Position update received from Unity3D for Veh: " << veh << " With NS3 ID: "<< Container.Get(veh)->GetId() <<  " Position" << " x: " <<  posit->Get(i)->x()<< " y: " << posit->Get(i)->y()<< " z: " << posit->Get(i)->z()<< " EEB is set to: "<< EmergencyBrake->Get(EmergencyBrake->Get(i)->VehID())->triggered() << std::endl;
        std::cout<< "Position update received from Unity3D for Veh: " << veh << " With NS3 ID: "<< Container.Get(veh)->GetId() <<  " Position" << " x: " <<  posit->Get(i)->x()<< " y: " << posit->Get(i)->y()<< " z: " << posit->Get(i)->z()<< std::endl;
    }

    std::cout<< "Unity3D time received: " << info->Get(0)->timeStamp() << " NS3 current time: "<< Simulator::Now() << std::endl;

    // If Unity3D simulation is about to stop and we recieved termination message
    if(termin){
        std::cout<< "Command to terminate received from Unity3D. Stopping simulation and closing NS3 thread now!" << std::endl;
        APIconnect_pub->close();
        APIconnect_sub->close();
        Simulator::Stop();
    }


    Simulator::Schedule (Seconds(FixedUpdInterval), &SetAllPositionFromAPI, Container, APIconnect, FixedUpdInterval, SpectrumModelPtr);

}



void ReceivedPacket(Ptr<const Packet> packet, Ptr<Node> rxNode, Ptr<Node> txNode) {
  /*
   * Callback invoked when the BSM is received by the VehicleNode.
   * The actual callback signature is defined in the bsm-application.cc/h
   * as BSMreceived TraceSource in factory.
  */

//  Get pointer to node's mobility model
  Ptr<MobilityModel> mob = rxNode->GetObject<MobilityModel>();
  double x = mob->GetPosition().x;
  double y = mob->GetPosition().y;
  double z = mob->GetPosition().z;

//  Get packet content
  uint8_t *buffer = new uint8_t[packet->GetSize ()];
  packet->CopyData(buffer,packet->GetSize());
  std::string packContent = std::string((char*)buffer);
  int br = packContent.compare("Brake");
  std::string ContentToSend;
//  Make a flatbuffer API structure
  flatbuffers::FlatBufferBuilder builder(1024);
  if (br == 0){
      ContentToSend = packContent;
    }
  else{
      ContentToSend = packContent;
    }

  auto msgCont = builder.CreateString(ContentToSend);

  auto PacInfoToSend = PacketInfo(txNode->GetId(), rxNode->GetId(), Simulator::Now().GetNanoSeconds(), false);
  auto RxPosition = Vec3Rx(x,z,y);
  auto bufferToAPI = CreateMsgRecAPI(builder, &RxPosition, &PacInfoToSend, msgCont);
  builder.Finish(bufferToAPI);

//  Sending structure to Unity3D using API.
  std::string topic = "fromNS";

  s_sendmore (*APIconnect_pub, topic);
  zmq::message_t msgToUnity(builder.GetSize());
  memcpy((void *)msgToUnity.data(), builder.GetBufferPointer(), builder.GetSize());
  auto res = APIconnect_pub->send(msgToUnity);


// Send to standard output stream to visually control API operation.
    std::string pac = std::string((char*)buffer) + " recieved by: VehNode:" + std::to_string(rxNode->GetId())+ " at time: " + std::to_string(Simulator::Now().GetSeconds());
    std::cout<<"BSM with content: "<< pac << " sent to Unity with result: " << res <<std::endl;
}


void SentPacket(Ptr<const Packet> packet, Ptr<Node> txNode) {
  /*
   * Callback invoked when the BSM is received by the VehicleNode.
   * The actual callback signature is defined in the bsm-application.cc/h
   * as BSMreceived TraceSource in factory.
  */

//  Get pointer to node's mobility model
  Ptr<MobilityModel> mob = txNode->GetObject<MobilityModel>();
  double x = mob->GetPosition().x;
  double y = mob->GetPosition().y;
  double z = mob->GetPosition().z;

//  Get packet content
  uint8_t *buffer = new uint8_t[packet->GetSize ()];
  packet->CopyData(buffer,packet->GetSize());
  std::string packContent = std::string((char*)buffer);

  std::string ContentToSend = packContent;

  flatbuffers::FlatBufferBuilder builder(1024);
  auto msgCont = builder.CreateString(ContentToSend);

  auto PacInfoToSend = PacketInfo(txNode->GetId(), 255, Simulator::Now().GetNanoSeconds(), true);
  auto TxPosition = Vec3Rx(x,z,y);
  auto bufferToAPI = CreateMsgRecAPI(builder, &TxPosition, &PacInfoToSend, msgCont);
  builder.Finish(bufferToAPI);
//  Sending structure to Unity3D using API.

  std::string topic = "fromNS";
  s_sendmore (*APIconnect_pub, topic);
  zmq::message_t msgToUnity(builder.GetSize());
  memcpy((void *)msgToUnity.data(), builder.GetBufferPointer(), builder.GetSize());
  auto res = APIconnect_pub->send(msgToUnity);


// Send to standard output stream to visually control API operation.
  std::string pac = std::string((char*)buffer) + " sent by: VehNode:" + std::to_string(txNode->GetId())+ " at time: " + std::to_string(Simulator::Now().GetSeconds());
  std::cout<<"BSM with content: "<< pac << " sent to Unity with result: " << res <<std::endl;
}


double DbmToW (double dbm)
{
  double mw = std::pow (10.0,dbm/10.0);
  return mw / 1000.0;
}

double WtodBm (double w)
{
  double dbm = std::log10 (w * 1000.0) * 10.0;
  return dbm;
}

int main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate6MbpsBW10MHz");
  uint32_t packetSize = 400; // bytes
  uint32_t numPackets = 10000;
  double interval = 0.1; // seconds
  double TxPower = 23; // TxPower in dBm
  TxPower = WtodBm(DbmToW(TxPower)*10); // Calculate TxPower PSD/MHz
//  bool verbose = false;




  CommandLine cmd;

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.Parse (argc, argv);

//  ConfigStore config;
//  config.ConfigureDefaults ();
//  LogComponentEnable ("Config", LOG_LEVEL_ALL);


//  GlobalValue::Bind ("SimulatorImplementationType",
//                     StringValue ("ns3::RealtimeSimulatorImpl"));


  // Convert to time object
  Time interPacketInterval = Seconds (interval);


  /*
   * We start the socket here to wait for the message from Unity3D to be synchronized
   * in the time domain: we want zero time to be apprx. the same in Unity3D and NS, after
   * which wall clock progressing at the same rate on both sides using RealTime simulator
   * implementation in NS3.
    */


    zmq::socket_t initSocket (context, ZMQ_REP);
    std::cout << "Connecting to Unity3D socket…" << std::endl;


    initSocket.bind ("tcp://127.0.0.1:5555");
    double UnityFixedUpd; // FixedUpdate step at which Unity3D updates Physics (our API update rate)
    double SimStart; // When we want to start simulating communication nodes
    int NumVeh; // Nodes expected in the Unity3D simulation
    bool UseGSCM; // GSCM channel model selector

    while (true){
        std::string updStep = s_recv (initSocket);
        std::string SimDelay = s_recv (initSocket);
        std::string VehNum = s_recv(initSocket);
        std::string ChModSel = s_recv(initSocket);

        if (ChModSel.compare("True") == 0){
            UseGSCM = true;
        } else
        {
            UseGSCM = false;
        }

        UnityFixedUpd = ::atof(updStep.c_str()); // converting string received back into double FixedUpdate
        SimStart = ::atof(SimDelay.c_str());
        NumVeh = ::atof(VehNum.c_str());

        if (NumVeh>0){
            break;
        }
    }
    std::cout << "Received Hello Message from Unity3D with FixedUpd value: " << UnityFixedUpd << std::endl;
    std::cout << "Simulation will start at: " << SimStart << std::endl;
    std::cout << "Number of nodes received: " << NumVeh << std::endl;
    std::string rplMSG = "HiUnity";
    s_send(initSocket, rplMSG);


    std::cout<<"NS3 process and API is running!"<<std::endl;
    std::cout<<"C-ITS GSCM WAVE (802.11p) scenario is chosen"<<std::endl;


  //Network Setup
  NodeContainer m_adhocTxNodes;
  m_adhocTxNodes.Create (NumVeh);


  // Spectrum
  SpectrumWifiPhyHelper spectrumPhy = SpectrumWifiPhyHelper::Default ();

  Ptr<SivertModelSpectrumChannel> spectrumChannel
      = CreateObject<SivertModelSpectrumChannel> ();

  Ptr<FixedRssLossModel> lossModel
      = CreateObject<FixedRssLossModel> ();

  spectrumChannel->AddPropagationLossModel (lossModel);

  Ptr<SivertSpectrumPropagationLossModel> SpectrumLossModel = CreateObject<SivertSpectrumPropagationLossModel> ();

  spectrumChannel->AddSpectrumPropagationLossModel(SpectrumLossModel);

  Ptr<ConstantSpeedPropagationDelayModel> delayModel
      = CreateObject<ConstantSpeedPropagationDelayModel> ();

  spectrumChannel->SetPropagationDelayModel (delayModel);


  std::cout<<"UseGSCM received from Unity: "<< UseGSCM <<std::endl;
  if (UseGSCM){
      Config::SetDefault ("ns3::SivertModelSpectrumChannel::UseGSCM", BooleanValue (true));
      Config::SetDefault ("ns3::SivertModelSpectrumChannel::UseGSCMpathLoss", BooleanValue (false));
      Config::SetDefault ("ns3::SivertModelSpectrumChannel::NumVeh", UintegerValue(NumVeh));
      Config::SetDefault ("ns3::FixedRssLossModel::Rss", DoubleValue(0.0));

    } else{
      Config::SetDefault ("ns3::SivertModelSpectrumChannel::UseGSCM", BooleanValue (false));
      Config::SetDefault ("ns3::SivertModelSpectrumChannel::UseGSCMpathLoss", BooleanValue (false));
    }

  Ptr<SivertSpectrumPropagationLossModel>  SpectrumModelPtr = SpectrumLossModel;



  spectrumPhy.SetChannel (spectrumChannel);
  spectrumPhy.SetErrorRateModel ("ns3::NistErrorRateModel");
  spectrumPhy.Set ("Frequency", UintegerValue (5990));
  spectrumPhy.Set ("TxPowerStart", DoubleValue (TxPower)); // dBm  (1.26 mW)
  spectrumPhy.Set ("TxPowerEnd", DoubleValue (TxPower));
  spectrumPhy.Set("ChannelWidth", UintegerValue(10));
  spectrumPhy.Set("EnergyDetectionThreshold", DoubleValue (-105));
  spectrumPhy.Set("CcaMode1Threshold", DoubleValue (-105));
  spectrumPhy.Set("RxNoiseFigure", DoubleValue (0.0));


//  // Setup WAVE PHY and MAC
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  WaveHelper waveHelper = WaveHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  wifi80211p.SetStandard(WIFI_PHY_STANDARD_80211_10MHZ);
//  wifi80211p.EnableLogComponents ();

  std::string m_phyMode ("OfdmRate6MbpsBW10MHz");

  // Setup 802.11p stuff
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (m_phyMode),
                                      "ControlMode",StringValue (m_phyMode));


  // Add an upper mac and disable rate control
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");
//  QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();

  // Setup net devices


  NetDeviceContainer m_adhocTxDevices;
  m_adhocTxDevices = wifi80211p.Install (spectrumPhy, wifi80211pMac, m_adhocTxNodes);


  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel"); // We use ConstantMobility to substitute the position of nodes at each FixedUpdate step in Unity3D
  mobility.Install (m_adhocTxNodes);

  InternetStackHelper internet;
  internet.Install (m_adhocTxNodes);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer m_adhocTxInterfaces = ipv4.Assign (m_adhocTxDevices);


  // BSM WAVE service configuration

  int chAccessMode = 0;
  WaveBsmHelper m_waveBsmHelper;
  uint32_t m_wavePacketSize = 400;
  double m_gpsAccuracyNs = 10;
  std::vector <double> m_txSafetyRanges{100, 200, 300, 400, 500 ,600, 700, 800, 900, 1000}; ///< list of ranges
  double m_txMaxDelayMs = 50;

  m_waveBsmHelper.InstallAPI (m_adhocTxInterfaces,
                           Seconds (3000),
                           m_wavePacketSize,
                           Seconds (0.1),
                           // GPS accuracy (i.e, clock drift), in number of ns
                           m_gpsAccuracyNs,
                           m_txSafetyRanges,
                           chAccessMode,
                           // tx max delay before transmit, in ms
                           MilliSeconds (m_txMaxDelayMs),
                           Seconds(SimStart)); // BSM app defined in a way BSM start is 1s delayed, so we compensate


  // Assign the callback for the BSM reception event.
  Config::ConnectWithoutContext("NodeList/*/ApplicationList/*/$ns3::BsmApplication/BSMreceived", MakeCallback(&ReceivedPacket));
  Config::ConnectWithoutContext("NodeList/*/ApplicationList/*/$ns3::BsmApplication/BSMsent", MakeCallback(&SentPacket));

  std::cout<<"Trying to establish API connection "<< std::endl;
  // Initiate PUB/SUB sockets for Unity3D API using the context.
  APIconnect_sub = s_sub_socket (context, "tcp://127.0.0.1:8002");
  APIconnect_pub = s_pub_socket (context, "tcp://127.0.0.1:8001");
  // Schedule the first API event



  if (UseGSCM){
      Simulator::Schedule (Seconds(SimStart - UnityFixedUpd), &SetAllPositionFromAPI, m_adhocTxNodes, APIconnect_sub, UnityFixedUpd, SpectrumModelPtr);
  } else{
      Simulator::Schedule (Seconds(SimStart - UnityFixedUpd), &SetAllPositionFromAPI, m_adhocTxNodes, APIconnect_sub, UnityFixedUpd);
  }



  // Start NS3 simulation
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
