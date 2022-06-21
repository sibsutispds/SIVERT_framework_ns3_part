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


#include "ns3/lte-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/lte-v2x-helper.h"
#include "ns3/config-store.h"
#include "ns3/lte-hex-grid-enb-topology-helper.h"
#include <ns3/buildings-helper.h>
#include <ns3/cni-urbanmicrocell-propagation-loss-model.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/spectrum-analyzer-helper.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/spectrum-model.h>
#include <ns3/sivert-spectrum-propagation-loss.h>
#include <ns3/sivert-model-spectrum-channel.h>
#include "ns3/ns2-mobility-helper.h"
#include <cfloat>
#include <sstream>

// Includes for SIVERT API
#include <zhelpers.hpp> // For ease we use zhelpers instead of zmq.hpp
#include <zmq.hpp>
#include "../flatbuffer_structures/positionSchema_generated.h" // include for our API's flatbuffer structure
#include "../flatbuffer_structures/MsgReceivedSchema_generated.h" // include for our API's flatbuffer structure

// General includes
#include <iostream>
#include <math.h>


using namespace SivertAPI::PosUpd;
using namespace SivertAPI::MsgReceived;
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("v2x_communication_mode_4");
//NS_LOG_COMPONENT_DEFINE ("SivertGSCM");

// Global vvariables for Pub and Sub ZeroMQ API connections
zmq::context_t context (1);
zmq::socket_t* APIconnect_sub;
zmq::socket_t* APIconnect_pub;
Ipv4InterfaceContainer ueIpIface;

// Output
std::string simtime = "log_simtime_v2x.csv";
std::string rx_data = "log_rx_data_v2x.csv";
std::string tx_data = "log_tx_data_v2x.csv";
std::string connections = "log_connections_v2x.csv";
std::string positions = "log_positions_v2x.csv";

Ptr<OutputStreamWrapper> log_connections;
Ptr<OutputStreamWrapper> log_simtime;
Ptr<OutputStreamWrapper> log_positions;
Ptr<OutputStreamWrapper> log_rx_data;
Ptr<OutputStreamWrapper> log_tx_data;

// Global variables
uint32_t ctr_totRx = 0; 	// Counter for total received packets
uint32_t ctr_totTx = 0; 	// Counter for total transmitted packets
uint16_t lenCam;
double baseline= 150.0;     // Baseline distance in meter (150m for urban, 320m for freeway)
uint16_t BeaconID = 0;
std::vector<bool> EBL_status;

// Responders users
NodeContainer ueVeh;


// Super IMPORTANT! We should use only one ZMQ context for our API!
static zmq::socket_t * s_sub_socket (zmq::context_t & context, std::string address) {

/*
 * For our needs one ZeroMQ context is enough (as a rule of thumb 1 context is required per 1 Gbit/s data exchange).
 * However, one could create numerous sockets using different ZeroMQ patterns for the API needs. Currently we use
 * Cleint/Server socket and PUB/SUB socket for our API.
*/

  zmq::socket_t * sck = new zmq::socket_t (context, ZMQ_SUB);
  sck->setsockopt( ZMQ_LINGER, 0 );             // ALWAYS, best before .bind()/.connect()
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
    auto EmergencyBrake = APIdata->CITS();



    for (unsigned int i = 0; i< EmergencyBrake->size(); i++){

        if (EmergencyBrake->Get(i)->triggered()) {

            try {
                EBL_status[EmergencyBrake->Get(i)->VehID()]  = true;
                std::cout<< "Node: "<< EmergencyBrake->Get(i)->VehID() << " EEB: " << EBL_status.at(EmergencyBrake->Get(i)->VehID()) << std::endl;
            } catch (const std::exception& e) {
                std::cout<< "ERROR:" << e.what() << std::endl;
            }

        }
    }


    for (unsigned int i = 0; i < posit->size() ; i++) {


        int veh = posit->Get(i)->id();
        Ptr<ConstantPositionMobilityModel> mobil = Container.Get(veh)->GetObject<ConstantPositionMobilityModel>();
        mobil->SetPosition(Vector(posit->Get(i)->x(), posit->Get(i)->z(), posit->Get(i)->y()));
        std::cout<< "Position update received from Unity3D for Veh: " << veh << " With NS3 ID: "<< Container.Get(veh)->GetId() <<  " Position" << " x: " <<  posit->Get(i)->x()<< " y: " << posit->Get(i)->y()<< " z: " << posit->Get(i)->z()<< std::endl;
    }

    std::cout<< "Unity3D time received: " << info->Get(0)->timeStamp() << " NS3 current time: "<< Simulator::Now() << std::endl;

    // If Unity3D simulation is about to stop and we received termination message
    if(termin){
        std::cout<< "Command to terminate received from Unity3D. Stopping simulation and closing NS3 thread now!" << std::endl;
        APIconnect_pub->close();
        APIconnect_sub->close();
        zmq_ctx_destroy(&context);

        Simulator::Stop();
    }


    Simulator::Schedule (Seconds(FixedUpdInterval), &SetAllPositionFromAPI, Container, APIconnect, FixedUpdInterval);

}


void SetAllDataFromAPI (NodeContainer Container, zmq::socket_t * APIconnect, double FixedUpdInterval, const Ptr<SivertSpectrumPropagationLossModel> SpectrumModelPtr)
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
//      std::cout<< "Spectrum ChID: " << ChID << " TxID: " << TxID << " RxID: " << RxID << std::endl;

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



  if (GSCMunity->UseGSCM()){
//      std::cout<< "GSCM bool: " << GSCMunity->UseGSCM() << " GSCM size: " << GSCMvec->size()<< std::endl;
      for(unsigned int l =0; l< GSCMvec->size(); l++){

          int ind = GSCMvec->size() - l - 1;
//          int ind = l;
//          std::cout<< "GSCM size " << GSCMvec->size()<< " index is: " << ind << std::endl;
          std::cout<< "RSS estimation GSCM for Tx: "<< GSCMvec->Get(ind)->Tx()<< " Rx: "<< GSCMvec->Get(ind)->Rx() << " from Unity to NS3 with: "<< GSCMvec->Get(ind)->rss() << " dBm/15kHz" << std::endl;


        }
    }

  for (unsigned int i = 0; i< EmergencyBrake->size(); i++){

      if (EmergencyBrake->Get(i)->triggered()) {

          try {
              EBL_status[EmergencyBrake->Get(i)->VehID()]  = true;
              std::cout<< "Node: "<< EmergencyBrake->Get(i)->VehID() << " EEB: " << EBL_status.at(EmergencyBrake->Get(i)->VehID()) << std::endl;
            } catch (const std::exception& e) {
              std::cout<< "ERROR:" << e.what() << std::endl;
            }

        }
    }


  for (unsigned int i = 0; i < posit->size() ; i++) {

      int veh = posit->Get(i)->id();
      Ptr<ConstantPositionMobilityModel> mobil = Container.Get(veh)->GetObject<ConstantPositionMobilityModel>();
      mobil->SetPosition(Vector(posit->Get(i)->x(), posit->Get(i)->z(), posit->Get(i)->y()));
      std::cout<< "Position update received from Unity3D for Veh: " << veh << " With NS3 ID: "<< Container.Get(veh)->GetId() <<  " Position" << " x: " <<  posit->Get(i)->x()<< " y: " << posit->Get(i)->y()<< " z: " << posit->Get(i)->z()<< std::endl;
    }

  std::cout<< "Unity3D time received: " << info->Get(0)->timeStamp() << " NS3 current time: "<< Simulator::Now() << std::endl;

  // If Unity3D simulation is about to stop and we recieved termination message
  if(termin){
      std::cout<< "Command to terminate received from Unity3D. Stopping simulation and closing NS3 thread now!" << std::endl;
      APIconnect_pub->close();
      APIconnect_sub->close();
      zmq_ctx_destroy(&context);
      Simulator::Stop();
    }


  Simulator::Schedule (Seconds(FixedUpdInterval), &SetAllDataFromAPI, Container, APIconnect, FixedUpdInterval, SpectrumModelPtr);

}


void
PrintStatus (uint32_t s_period, Ptr<OutputStreamWrapper> log_simtime)
{
    if (ctr_totRx > ctr_totTx)
    {
        ctr_totRx = ctr_totTx;
    }
        *log_simtime->GetStream() << Simulator::Now ().GetSeconds () << ";" << ctr_totRx << ";" << ctr_totTx << ";" << (double) ctr_totRx / ctr_totTx << std::endl;

    Simulator::Schedule(Seconds(s_period), &PrintStatus, s_period,log_simtime);
}

void
SidelinkV2xAnnouncementMacTrace(Ptr<Socket> socket)
{
    Ptr <Node> node = socket->GetNode();
    int id = node->GetId();
    uint32_t simTime = Simulator::Now().GetMilliSeconds();
    Ptr<MobilityModel> posMobility = node->GetObject<MobilityModel>();
    Vector posTx = posMobility->GetPosition();

    // check for each UE distance to transmitter
    for (uint8_t i=0; i<ueVeh.GetN();i++)
    {
        Ptr<MobilityModel> mob = ueVeh.Get(i)->GetObject<MobilityModel>();
        Vector posRx = mob->GetPosition();

        double distance = sqrt(pow((posTx.x - posRx.x),2.0)+pow((posTx.y - posRx.y),2.0));
        if  (distance > 0 && distance <= baseline)
        {
            ctr_totTx++;
        }
    }
    // Generate CAM
    std::ostringstream msgCam;
//    msgCam << id-1 << ";" << simTime << ";" << (int) posTx.x << ";" << (int) posTx.y << '\0';
//    std::cout << "Current EBL content: "<< std::endl;
//    for (auto i = EBL_status.begin(); i != EBL_status.end(); ++i){
//        std::cout << *i << ' ';
//      }

    if (EBL_status.at(id)){
        msgCam << "Brake" << id;
      }
    else{
        msgCam<< "BeaconID: "<< BeaconID <<" VehID: "<< id << " Position x: "<< posTx.x << " y: "<< posTx.y << " z: "<< posTx.z << " Time: "<< Simulator::Now();
      }
    std::cout<< "Vehicle: " << id << " sends message with content: " << msgCam.str() << std::endl;
    Ptr<Packet> packet = Create<Packet>((uint8_t*)msgCam.str().c_str(),lenCam);
    socket->Send(packet);
    *log_tx_data->GetStream() << ctr_totTx << ";" << simTime << ";"  << id << ";" << (int) posTx.x << ";" << (int) posTx.y << std::endl;
    BeaconID++;

  flatbuffers::FlatBufferBuilder builder(1024);
  auto msgCont = builder.CreateString(msgCam.str().c_str());
  auto PacInfoToSend = PacketInfo(id, 255, Simulator::Now().GetNanoSeconds(), true);
  auto TxPosition = Vec3Rx(posTx.x,posTx.z,posTx.y);
  auto bufferToAPI = CreateMsgRecAPI(builder, &TxPosition, &PacInfoToSend, msgCont);
  builder.Finish(bufferToAPI);
//  Sending structure to Unity3D using API.
  std::string topic = "fromNS";
  s_sendmore (*APIconnect_pub, topic);
  zmq::message_t msgToUnity(builder.GetSize());
  memcpy((void *)msgToUnity.data(), builder.GetBufferPointer(), builder.GetSize());
  auto res = APIconnect_pub->send(msgToUnity);

  std::string pac = msgCam.str() + " sent by: VehNode:" + std::to_string(id)+ " at time: " + std::to_string(Simulator::Now().GetSeconds());
  std::cout<<"BSM with content: "<< pac << " sent to Unity with result: " << res <<std::endl;

}

static void
ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Node> rxNode = socket->GetNode();
    Ptr<MobilityModel> posMobility = rxNode->GetObject<MobilityModel>();
    Vector posRx = posMobility->GetPosition();
    Ptr<Packet> packet = socket->Recv ();
    uint8_t *buffer = new uint8_t[packet->GetSize()];
    packet->CopyData(buffer,packet->GetSize());
    std::string s = std::string((char*)buffer);

    size_t pos = 0;
    std::string copy = s;
    std::string token;
    int TxID;

    if (copy.find("Brake") == 0){
        pos = copy.find("Brake");
        token = copy.substr(pos+5,1);
        TxID = atoi(token.c_str());
      }
    else{
        pos = copy.find("VehID: ");
        token = copy.substr(pos+7,1);
        TxID = atoi(token.c_str());
      }


//    std::cout << "Substring position: " << pos << " Token copied: "<< token <<" TxID retrieved is: " << TxID << std::endl;


  //  Get pointer to node's mobility model
  double x = posMobility->GetPosition().x;
  double y = posMobility->GetPosition().y;
  double z = posMobility->GetPosition().z;
//  std::cout<< "Trying to extract Tx node Ptr..."<< "TxID: " << TxID <<std::endl;
  std::pair<Ptr<Ipv4>, uint32_t> interface = ueIpIface.Get (TxID);
  Ptr<Ipv4> pp = interface.first;
  Ptr<Node> txNode = pp->GetObject<Node> ();


  std::string packContent = std::string((char*)buffer);
  int br = packContent.compare("Brake");
  std::string ContentToSend;
//  Make a flatbuffer API structure
  flatbuffers::FlatBufferBuilder builder(1024);
  if (br == 0){
      ContentToSend = "Brake";
//      ContentToSend = "Beacon recieved by: VehNode:" + std::to_string(rxNode->GetId())+ " From vehicle: " + std::to_string(TxID)  + " at time: " + std::to_string(Simulator::Now().GetSeconds());
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
  APIconnect_pub->send(msgToUnity,0);

// Send to standard output stream to visually control API operation.
  std::string pac = std::string((char*)buffer) + " recieved by: VehNode: " + std::to_string(rxNode->GetId())+ " at time: " + std::to_string(Simulator::Now().GetSeconds());
  std::cout<<"BSM with content: "<< pac <<std::endl;

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


int
main (int argc, char *argv[])
{

  /*
   * We start the socket here to wait for the message from Unity3D to be synchronized
   * in the time domain: we want zero time to be the same in Unity3D and NS, after
   * which wall clock progressing at the same rate on both sides using RealTime simulator
   * implementation in NS3.
    */

//    zmq::context_t context (1); // NB: there should be only one context in one thread --> thus, SINGLE context for entire NS3 simulation.
    zmq::socket_t initSocket (context, ZMQ_REP);
    std::cout << "Connecting to Unity3D socket…" << std::endl;


    initSocket.bind ("tcp://127.0.0.1:5555");
    int attempts = 1;
    double UnityFixedUpd; // FixedUpdate step at which Unity3D updates Physics (our API update rate)
    double SimStart; // When we want to start simulating communication nodes
    int NumNodes; // Nodes expected in the Unity3D simulation
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
        NumNodes = ::atof(VehNum.c_str());

        if (NumNodes>0){
            break;
          }
    }
    std::cout << "Received Hello Message from Unity3D with FixedUpd value: " << UnityFixedUpd << std::endl;
    std::cout << "Simulation will start at: " << SimStart << std::endl;
    std::string rplMSG = "HiUnity";
    s_send(initSocket, rplMSG);

    for (int j = 0; j < NumNodes; ++j)
      {
        EBL_status.push_back(false);
      }

    initSocket.close(); // closing irrelevant socket

    std::cout<<"NS3 process and API is running!"<<std::endl;
    std::cout<<"LTE-V2X scenario is chosen"<<std::endl;



    LogComponentEnable ("v2x_communication_mode_4", LOG_INFO);

    // Initialize some values
    // NOTE: commandline parser is currently (05.04.2019) not working for uint8_t (Bug 2916)

    uint16_t simTime = 1000;                 // Simulation time in seconds
    uint32_t numVeh = NumNodes;                  // Number of vehicles
    lenCam = 400;                           // Length of CAM message in bytes [50-300 Bytes]
    double ueTxPower = 23.0;                // Transmission power in dBm
    double probResourceKeep = 0.0;          // Probability to select the previous resource again [0.0-0.8]
    uint32_t mcs = 20;                      // Modulation and Coding Scheme
    bool harqEnabled = false;               // Retransmission enabled
    bool adjacencyPscchPssch = true;        // Subchannelization scheme
    bool partialSensing = false;            // Partial sensing enabled (actual only partialSensing is false supported)
    uint16_t sizeSubchannel = 10;           // Number of RBs per subchannel
    uint16_t numSubchannel = 3;             // Number of subchannels per subframe
    uint16_t startRbSubchannel = 0;         // Index of first RB corresponding to subchannelization
    uint16_t pRsvp = 100;                   // Resource reservation interval
    uint16_t t1 = 4;                        // T1 value of selection window
    uint16_t t2 = 100;                      // T2 value of selection window
    uint16_t slBandwidth;                   // Sidelink bandwidth
    std::string tracefile;                  // Name of the tracefile


   ueTxPower = WtodBm(DbmToW(ueTxPower)*(numSubchannel*sizeSubchannel*180e3/1e6)); // Calculate TxPower PSD/MHz
   std::cout<< "Tx power calculated: " << ueTxPower << std::endl;


  // Command line arguments
    CommandLine cmd;
    cmd.AddValue ("time", "Simulation Time", simTime);
    cmd.AddValue ("numVeh", "Number of Vehicles", numVeh);
    cmd.AddValue ("adjacencyPscchPssch", "Scheme for subchannelization", adjacencyPscchPssch);
    cmd.AddValue ("sizeSubchannel", "Number of RBs per Subchannel", sizeSubchannel);
    cmd.AddValue ("numSubchannel", "Number of Subchannels", numSubchannel);
    cmd.AddValue ("startRbSubchannel", "Index of first subchannel index", startRbSubchannel);
    cmd.AddValue ("T1", "T1 Value of Selection Window", t1);
    cmd.AddValue ("T2", "T2 Value of Selection Window", t2);
    //cmd.AddValue ("harqEnabled", "HARQ Retransmission Enabled", harqEnabled);
    //cmd.AddValue ("partialSensingEnabled", "Partial Sensing Enabled", partialSensing);
    cmd.AddValue ("lenCam", "Packetsize in Bytes", lenCam);
    cmd.AddValue ("mcs", "Modulation and Coding Scheme", mcs);
    cmd.AddValue ("pRsvp", "Resource Reservation Interval", pRsvp);
    cmd.AddValue ("probResourceKeep", "Probability for selecting previous resource again", probResourceKeep);
    cmd.AddValue ("log_simtime", "name of the simtime logfile", simtime);
    cmd.AddValue ("log_rx_data", "name of the rx data logfile", rx_data);
    cmd.AddValue ("log_tx_data", "name of the tx data logfile", tx_data);
    cmd.AddValue ("tracefile", "Path of ns-3 tracefile", tracefile);
    cmd.AddValue ("baseline", "Distance in which messages are transmitted and must be received", baseline);
    cmd.Parse (argc, argv);

//    //
//    // But since this is a realtime script, don't allow the user to mess with
//    // that.
//    //
//    GlobalValue::Bind ("SimulatorImplementationType",
//                       StringValue ("ns3::RealtimeSimulatorImpl"));


    AsciiTraceHelper ascii;
    log_simtime = ascii.CreateFileStream(simtime);
    log_rx_data = ascii.CreateFileStream(rx_data);
    log_tx_data = ascii.CreateFileStream(tx_data);
    log_connections = ascii.CreateFileStream(connections);
    log_positions = ascii.CreateFileStream(positions);

    NS_LOG_INFO ("Starting network configuration...");

    // Set the UEs power in dBm
    Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePhy::RsrpUeMeasThreshold", DoubleValue (-10.0));
    // Enable V2X communication on PHY layer
    Config::SetDefault ("ns3::LteUePhy::EnableV2x", BooleanValue (true));

    // Set power
    Config::SetDefault ("ns3::LteUePowerControl::Pcmax", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePowerControl::PsschTxPower", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePowerControl::PscchTxPower", DoubleValue (ueTxPower));

    if (adjacencyPscchPssch)
    {
        slBandwidth = sizeSubchannel * numSubchannel;
    }
    else
    {
        slBandwidth = (sizeSubchannel+2) * numSubchannel;
    }

    // Configure for UE selected
    Config::SetDefault ("ns3::LteUeMac::UlBandwidth", UintegerValue(slBandwidth));
    Config::SetDefault ("ns3::LteUeMac::EnableV2xHarq", BooleanValue(harqEnabled));
    Config::SetDefault ("ns3::LteUeMac::EnableAdjacencyPscchPssch", BooleanValue(adjacencyPscchPssch));
    Config::SetDefault ("ns3::LteUeMac::EnablePartialSensing", BooleanValue(partialSensing));
    Config::SetDefault ("ns3::LteUeMac::SlGrantMcs", UintegerValue(mcs));
    Config::SetDefault ("ns3::LteUeMac::SlSubchannelSize", UintegerValue (sizeSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlSubchannelNum", UintegerValue (numSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlStartRbSubchannel", UintegerValue (startRbSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlPrsvp", UintegerValue(pRsvp));
    Config::SetDefault ("ns3::LteUeMac::SlProbResourceKeep", DoubleValue(probResourceKeep));
    Config::SetDefault ("ns3::LteUeMac::SelectionWindowT1", UintegerValue(t1));
    Config::SetDefault ("ns3::LteUeMac::SelectionWindowT2", UintegerValue(t2));
    //Config::SetDefault ("ns3::LteUeMac::EnableExcludeSubframe", BooleanValue(excludeSubframe));
    Config::SetDefault ("ns3::LteUeRrc::MinSrsrp", DoubleValue(-125));
    Config::SetDefault ("ns3::LteUePhy::MinSrsrp", DoubleValue(-125));

    Config::SetDefault("ns3::LteUePhy::NoiseFigure", DoubleValue (0)); // Added noise figure for SINR
    Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
    Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (true));

    Config::SetDefault ("ns3::LteSpectrumPhy::SlDataBLERModelEnabled", BooleanValue (true));
    Config::SetDefault ("ns3::LteSpectrumPhy::NistErrorModelEnabled", BooleanValue (true));

    if (UseGSCM){
        Config::SetDefault ("ns3::SivertModelSpectrumChannel::UseGSCM", BooleanValue (true));
        Config::SetDefault ("ns3::SivertModelSpectrumChannel::UseGSCMpathLoss", BooleanValue (false));
        Config::SetDefault ("ns3::SivertModelSpectrumChannel::NumVeh", UintegerValue(numVeh));
        Config::SetDefault ("ns3::FixedRssLossModel::Rss", DoubleValue(0.0));
    } else{
        Config::SetDefault ("ns3::SivertModelSpectrumChannel::UseGSCM", BooleanValue (false));
        Config::SetDefault ("ns3::SivertModelSpectrumChannel::UseGSCMpathLoss", BooleanValue (false));
    }
//    Config::SetDefault ("ns3::LteHelper::UseGSCM", BooleanValue (true));

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();
//    LogComponentEnableAll(LOG_LEVEL_FUNCTION);
    //    ns3::LogComponentEnableAll(NS_LOG_DEBUG)

  // Create node container to hold all UEs
    NodeContainer ueAllNodes;

    NS_LOG_INFO ("Installing Mobility Model...");

    if (tracefile.empty())
    {
        // Create nodes
        ueVeh.Create (numVeh);
        ueAllNodes.Add (ueVeh);

        // Install constant random positions
        MobilityHelper mobVeh;
        mobVeh.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        Ptr<ListPositionAllocator> staticVeh[ueVeh.GetN()];
        for (uint16_t i=0; i<ueVeh.GetN();i++)
        {
            staticVeh[i] = CreateObject<ListPositionAllocator>();
            staticVeh[i]->Add(Vector(0,0,0));
            mobVeh.SetPositionAllocator(staticVeh[i]);
            mobVeh.Install(ueVeh.Get(i));
        }
    }
    else
    {
        // Create nodes
        ueVeh.Create (numVeh);
        ueAllNodes.Add (ueVeh);

//        Ns2MobilityHelper ns2 = Ns2MobilityHelper(tracefile);
//        ns2.Install();
    }


//    NS_LOG_INFO ("Creating helpers...");
    // EPC
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // LTE Helper
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->DisableNewEnbPhy(); // Disable eNBs for out-of-coverage modelling

    // V2X
    Ptr<LteV2xHelper> lteV2xHelper = CreateObject<LteV2xHelper> ();
    lteV2xHelper->SetLteHelper (lteHelper);

    // Configure eNBs' antenna parameters before deploying them.
    lteHelper->SetEnbAntennaModelType ("ns3::NistParabolic3dAntennaModel");

    Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", StringValue ("55140"));
    lteHelper->SetAttribute ("UseSameUlDlPropagationCondition", BooleanValue(true));
    // Configure V2X stations channel model

    lteHelper->SetSpectrumChannelType("ns3::SivertModelSpectrumChannel");
    if (UseGSCM){
        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FixedRssLossModel")); // Set PathLoss
        // to 0 dB - path Loss will be calculated by SpectrumModel where we pass data from Unity3D GSCM
        lteHelper->SetAttribute("FadingModel", StringValue ("ns3::SivertSpectrumPropagationLossModel"));
        // Set SpectrumModel to SIVERT to plug gain values calculated in Unity3D GSCM
        lteHelper->SetUeAntennaModelType("ns3::IsotropicAntennaModel"); // Important to set Isotropic
        // antenna here, because antenna rariation pattern is already included in SIVERT GSCM Unity3D
    } else{
        lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::CniUrbanmicrocellPropagationLossModel"));
    }
    std::cout<<"UseGSCM received from Unity: "<< UseGSCM <<std::endl;




    // Create eNB Container
    NodeContainer eNodeB;
    eNodeB.Create(1);

    // Topology eNodeB
    Ptr<ListPositionAllocator> pos_eNB = CreateObject<ListPositionAllocator>();
    pos_eNB->Add(Vector(170,508,15));

    //  Install mobility eNodeB
    MobilityHelper mob_eNB;
    mob_eNB.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob_eNB.SetPositionAllocator(pos_eNB);
    mob_eNB.Install(eNodeB);

    // Install Service
    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(eNodeB);

    // Required to use NIST 3GPP model
    BuildingsHelper::Install (eNodeB);
    BuildingsHelper::Install (ueAllNodes);
    BuildingsHelper::MakeMobilityModelConsistent ();

    // Install LTE devices to all UEs
//    NS_LOG_INFO ("Installing UE's network devices...");
    lteHelper->SetAttribute("UseSidelink", BooleanValue (true));
    NetDeviceContainer ueRespondersDevs = lteHelper->InstallUeDevice (ueVeh);
    NetDeviceContainer ueDevs;
    ueDevs.Add (ueRespondersDevs);

    // Install the IP stack on the UEs
//    NS_LOG_INFO ("Installing IP stack...");
    InternetStackHelper internet;
    internet.Install (ueAllNodes);

    // Assign IP adress to UEs
//    NS_LOG_INFO ("Allocating IP addresses and setting up network route...");
//    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address (ueDevs);
    Ipv4StaticRoutingHelper Ipv4RoutingHelper;

    for(uint32_t u = 0; u < ueAllNodes.GetN(); ++u)
        {
            Ptr<Node> ueNode = ueAllNodes.Get(u);
            // Set the default gateway for the UE
            Ptr<Ipv4StaticRouting> ueStaticRouting = Ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
            ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress(), 1);
        }

//    NS_LOG_INFO("Attaching UE's to LTE network...");
    //Attach each UE to the best available eNB
    lteHelper->Attach(ueDevs);

//    NS_LOG_INFO ("Creating sidelink groups...");
    std::vector<NetDeviceContainer> txGroups;
    txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueRespondersDevs, numVeh);

    lteV2xHelper->PrintGroups(txGroups);
    // compute average number of receivers associated per transmitter and vice versa
    double totalRxs = 0;
    std::map<uint32_t, uint32_t> txPerUeMap;
    std::map<uint32_t, uint32_t> groupsPerUe;

    std::vector<NetDeviceContainer>::iterator gIt;
    for(gIt=txGroups.begin(); gIt != txGroups.end(); gIt++)
        {
            uint32_t numDevs = gIt->GetN();

            totalRxs += numDevs-1;
            uint32_t nId;

            for(uint32_t i=1; i< numDevs; i++)
                {
                    nId = gIt->Get(i)->GetNode()->GetId();
                    txPerUeMap[nId]++;
                }
        }

    double totalTxPerUe = 0;
    std::map<uint32_t, uint32_t>::iterator mIt;
    for(mIt=txPerUeMap.begin(); mIt != txPerUeMap.end(); mIt++)
        {
            totalTxPerUe += mIt->second;
            groupsPerUe [mIt->second]++;
        }

    // lteV2xHelper->PrintGroups (txGroups, log_connections);

//    NS_LOG_INFO ("Installing applications...");

    // Application Setup for Responders
    std::vector<uint32_t> groupL2Addresses;
    uint32_t groupL2Address = 0x00;
    Ipv4AddressGenerator::Init(Ipv4Address ("225.0.0.0"), Ipv4Mask("255.0.0.0"));
    Ipv4Address clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));

    uint16_t application_port = 8000; // Application port to TX/RX
    NetDeviceContainer activeTxUes;



    for(gIt=txGroups.begin(); gIt != txGroups.end(); gIt++)
        {
            // Create Sidelink bearers
            // Use Tx for the group transmitter and Rx for all the receivers
            // Split Tx/Rx

            NetDeviceContainer txUe ((*gIt).Get(0));
            activeTxUes.Add(txUe);
            NetDeviceContainer rxUes = lteV2xHelper->RemoveNetDevice ((*gIt), txUe.Get (0));
            Ptr<LteSlTft> tft = Create<LteSlTft> (LteSlTft::TRANSMIT, clientRespondersAddress, groupL2Address);
            lteV2xHelper->ActivateSidelinkBearer (Seconds(SimStart), txUe, tft);
            tft = Create<LteSlTft> (LteSlTft::RECEIVE, clientRespondersAddress, groupL2Address);
            lteV2xHelper->ActivateSidelinkBearer (Seconds(SimStart), rxUes, tft);

            //std::cout << "Created group L2Address=" << groupL2Address << " IPAddress=";
            //clientRespondersAddress.Print(std::cout);
            //std::cout << std::endl;

            //Individual Socket Traffic Broadcast everyone
            Ptr<Socket> host = Socket::CreateSocket(txUe.Get(0)->GetNode(),TypeId::LookupByName ("ns3::UdpSocketFactory"));
            host->Bind();
            host->Connect(InetSocketAddress(clientRespondersAddress,application_port));
            host->SetAllowBroadcast(true);
            host->ShutdownRecv();

            //Ptr<LteUeRrc> ueRrc = DynamicCast<LteUeRrc>( txUe.Get (0)->GetObject<LteUeNetDevice> ()->GetRrc () );
            //ueRrc->TraceConnectWithoutContext ("SidelinkV2xMonitoring", MakeBoundCallback (&SidelinkV2xMonitoringTrace, stream));
            //oss << txUe.Get(0) ->GetObject<LteUeNetDevice>()->GetImsi();
            //Ptr<LteUePhy> uePhy = DynamicCast<LteUePhy>( txUe.Get (0)->GetObject<LteUeNetDevice> ()->GetPhy () );
            //uePhy->TraceConnect ("SidelinkV2xAnnouncement", oss.str() ,MakeBoundCallback (&SidelinkV2xAnnouncementPhyTrace, stream1));
            //uePhy->TraceConnectWithoutContext ("SidelinkV2xAnnouncement", MakeBoundCallback (&SidelinkV2xAnnouncementPhyTrace, host));
            Ptr<LteUeMac> ueMac = DynamicCast<LteUeMac>( txUe.Get (0)->GetObject<LteUeNetDevice> ()->GetMac () );
            ueMac->TraceConnectWithoutContext ("SidelinkV2xAnnouncement", MakeBoundCallback (&SidelinkV2xAnnouncementMacTrace, host));
            //ueMac->TraceConnect ("SidelinkV2xAnnouncement", oss.str() ,MakeBoundCallback (&SidelinkV2xAnnouncementMacTrace, stream2));

            Ptr<Socket> sink = Socket::CreateSocket(txUe.Get(0)->GetNode(),TypeId::LookupByName ("ns3::UdpSocketFactory"));
            sink->Bind(InetSocketAddress (Ipv4Address::GetAny (), application_port));
            sink->SetRecvCallback (MakeCallback (&ReceivePacket));

            //store and increment addresses
            groupL2Addresses.push_back (groupL2Address);
            groupL2Address++;
            clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));
        }

        NS_LOG_INFO ("Creating Sidelink Configuration...");
        Ptr<LteUeRrcSl> ueSidelinkConfiguration = CreateObject<LteUeRrcSl>();
        ueSidelinkConfiguration->SetSlEnabled(true);
        ueSidelinkConfiguration->SetV2xEnabled(true);

        LteRrcSap::SlV2xPreconfiguration preconfiguration;
//        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.carrierFreq = 54890;
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.carrierFreq = 55140; // EARFCN 55140 corresponds to 5915.00 MHz
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.slBandwidth = slBandwidth;

        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.nbPools = 1;
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.nbPools = 1;

        SlV2xPreconfigPoolFactory pFactory;
        pFactory.SetHaveUeSelectedResourceConfig (true);
        pFactory.SetSlSubframe (std::bitset<20> (0xFFFFF));
        pFactory.SetAdjacencyPscchPssch (adjacencyPscchPssch);
        pFactory.SetSizeSubchannel (sizeSubchannel);
        pFactory.SetNumSubchannel (numSubchannel);
        pFactory.SetStartRbSubchannel (startRbSubchannel);
        pFactory.SetStartRbPscchPool (0);
        pFactory.SetDataTxP0 (-4);
        pFactory.SetDataTxAlpha (0.9);

        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0] = pFactory.CreatePool ();
        preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0] = pFactory.CreatePool ();
        ueSidelinkConfiguration->SetSlV2xPreconfiguration (preconfiguration);

        // Print Configuration
        *log_rx_data->GetStream() << "RxPackets;RxTime;RxId;TxId;TxTime;xPos;yPos" << std::endl;
        *log_tx_data->GetStream() << "TxPackets;TxTime;TxId;xPos;yPos" << std::endl;

        NS_LOG_INFO ("Installing Sidelink Configuration...");
        lteHelper->InstallSidelinkV2xConfiguration (ueRespondersDevs, ueSidelinkConfiguration);

//        NS_LOG_INFO ("Enabling LTE traces...");

//        lteHelper->EnableTraces();
//        lteHelper->EnableLogComponents();
//        LogComponentEnable ("TraceFadingLossModel", LOG_LEVEL_ALL);
//        LogComponentEnable ("LteGlobalPathlossDatabase", LOG_LEVEL_ALL);
//        LogComponentEnable ("MacStatsCalculator", LOG_LEVEL_ALL);
//        LogComponentEnable ("PhyRxStatsCalculator", LOG_LEVEL_ALL);
//        LogComponentEnable ("PhyStatsCalculator", LOG_LEVEL_ALL);
//        LogComponentEnable ("PhyTxStatsCalculator", LOG_LEVEL_ALL);
//        LogComponentEnable ("LteSpectrumPhy", LOG_LEVEL_ALL);
//        LogComponentEnable ("LtePhyErrorModel", LOG_LEVEL_ALL);

        *log_simtime->GetStream() << "Simtime;TotalRx;TotalTx;PRR" << std::endl;
        Simulator::Schedule(Seconds(SimStart), &PrintStatus, 1, log_simtime);

        // Getting Pointer to Spectrum Channel model
        Ptr<SivertSpectrumPropagationLossModel>  SpectrumModelPtr;
        if (UseGSCM){
            const Ptr<SpectrumChannel> SpectrumCh_tmp = lteHelper->GetDownlinkSpectrumChannel();
            std::cout << "Spectrum channel: " << SpectrumCh_tmp << std::endl;
            const Ptr<SivertModelSpectrumChannel> SpectrumChPtr = DynamicCast<SivertModelSpectrumChannel> (SpectrumCh_tmp);
            std::cout << "Spectrum channel: " << SpectrumChPtr << std::endl;
            NS_LOG_INFO ("Getting spectrum pointers...");
            const Ptr<SpectrumPropagationLossModel> SpectrumModelPtr_temp = SpectrumChPtr->GetSpectrumPropagationLossModel();
            NS_LOG_INFO ("Getting spectrum pointers 2...");
            SpectrumModelPtr = DynamicCast<SivertSpectrumPropagationLossModel> (SpectrumModelPtr_temp);
        }




    NS_LOG_INFO ("Starting Simulation...");
//        Simulator::Stop(MilliSeconds(simTime*1000+40));

        // Initiate PUB/SUB sockets for Unity3D API using the context.
        APIconnect_sub = s_sub_socket (context, "tcp://127.0.0.1:8002");
        APIconnect_pub = s_pub_socket (context, "tcp://127.0.0.1:8001");
        // Scedule the first API event
        if (UseGSCM){
            Simulator::Schedule (Seconds(SimStart - UnityFixedUpd), &SetAllDataFromAPI, ueVeh, APIconnect_sub, UnityFixedUpd, SpectrumModelPtr);
        } else{
            Simulator::Schedule (Seconds(SimStart - UnityFixedUpd), &SetAllPositionFromAPI, ueVeh, APIconnect_sub, UnityFixedUpd);
        }

        Simulator::Run();

        Simulator::Destroy();

        NS_LOG_INFO("Simulation done.");
        return 0;
}
