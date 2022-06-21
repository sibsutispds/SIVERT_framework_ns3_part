/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 CTTC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Nicola Baldo <nbaldo@cttc.es>
 */

#include <ns3/mobility-model.h>
#include <ns3/sivert-spectrum-propagation-loss.h>
#include <ns3/uinteger.h>


namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (SivertSpectrumPropagationLossModel);


SivertSpectrumPropagationLossModel::~SivertSpectrumPropagationLossModel ()
{
}

SivertSpectrumPropagationLossModel::SivertSpectrumPropagationLossModel()
{
}

void SivertSpectrumPropagationLossModel::DoInitialize()
{
  double init_gain = 1e-5;
  std::cout<< "Num of Sub-channels: " << m_number_of_subchannels << std::endl;
  for (int i = 0; i < 100 ; ++i) {
      for (int j = 0; j < 100; ++j) {
          for (int k = 0; k < m_number_of_subchannels; ++k)
            {
              m_Channel_from_Unity3D[i][j].push_back(1);
            }
        }
    }
}


TypeId
SivertSpectrumPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SivertSpectrumPropagationLossModel")
    .SetParent<SpectrumPropagationLossModel> ()
    .SetGroupName ("Spectrum")
    .AddConstructor<SivertSpectrumPropagationLossModel> ()
    .AddAttribute("NumFreq",
                  "Number of sub-channels",
                  UintegerValue (30),
                  MakeUintegerAccessor(&SivertSpectrumPropagationLossModel::m_number_of_subchannels),
                  MakeUintegerChecker<uint16_t> (1, 200))
  ;
  return tid;
}



Ptr<SpectrumValue>
SivertSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity (Ptr<const SpectrumValue> txPsd,
                                                                 Ptr<const MobilityModel> a,
                                                                 Ptr<const MobilityModel> b) const
{
  Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue> (txPsd);
  Values::iterator vit = rxPsd->ValuesBegin ();
  Bands::const_iterator fit = rxPsd->ConstBandsBegin ();

  NS_ASSERT (a);
  NS_ASSERT (b);


  Values::iterator first_RB_index = rxPsd->ValuesBegin ();

  double d = a->GetDistanceFrom (b);

  while (vit != rxPsd->ValuesEnd ())
    {
      BandInfo fr = *fit;
      int RBnumber = vit - first_RB_index;
      NS_ASSERT (fit != rxPsd->ConstBandsEnd ());
//      std::cout<<"RB number: " << RBnumber << std::endl;
//      std::cout<<"RB frequency: " << fr.fc << std::endl;
//      std::cout<<"Sivert Spectrum PSD Value before: " << *vit << std::endl;
      *vit /= CalculateLoss (fit->fc, d); // Prx = Ptx / loss
//      std::cout<<"Sivert Spectrum PSD Value after: " << *vit << std::endl;
      ++vit;
      ++fit;
    }
  return rxPsd;
}


double
SivertSpectrumPropagationLossModel::CalculateLoss (double f, double d) const
{
  NS_ASSERT (d >= 0);

  if (d == 0)
    {
      return 1;
    }

  NS_ASSERT (f > 0);
  double loss_sqrt = (4 * M_PI * f * d) / 3e8;
  double loss = loss_sqrt * loss_sqrt;

  if (loss < 1)
    {
      loss = 1;
    }
  return loss;
}


Ptr<SpectrumValue>
SivertSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensitySIVERT (Ptr<const SpectrumValue> txPsd,
                                                                  int TxID,
                                                                  int RxID) const
{


  Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue> (txPsd);
  Values::iterator vit = rxPsd->ValuesBegin ();
  Bands::const_iterator fit = rxPsd->ConstBandsBegin ();
  std::vector<double> psd_vector = m_Channel_from_Unity3D[TxID][RxID];
  std::cout<< "DoCalcRxPowerSpectralDensitySIVERT input: TxID: "<< TxID << " RxID: "<< RxID<< " PSD size: " << std::endl;
//  std::cout<< "m_Channel_from_Unity3 vector size: "<< psd_vector.size() << std::endl;
  Values::iterator first_RB_index = rxPsd->ValuesBegin ();

//  std::cout<< "rxPSD size: "<< rxPsd->ValuesBegin() - rxPsd->ValuesEnd() << std::endl;

  while (vit != rxPsd->ValuesEnd ())
    {
      if (TxID!=RxID){
          BandInfo fr = *fit;
          int RBnumber = vit - first_RB_index;
          NS_ASSERT (fit != rxPsd->ConstBandsEnd ());
//          std::cout<<"RB number: " << RBnumber << std::endl;
//          std::cout<<"RB frequency: " << fr.fc << std::endl;
//          std::cout<<"Sivert Spectrum PSD Value before: " << *vit << std::endl;
          *vit *= psd_vector[RBnumber]; // Prx = Ptx / loss
//          std::cout<<"Sivert Spectrum PSD Value after: " << *vit << std::endl;
//          std::cout<<"PSD gain received from Unity3D: " << psd_vector[RBnumber] << std::endl;
          ++vit;
          ++fit;
        }
      else{
          BandInfo fr = *fit;
          int RBnumber = vit - first_RB_index;
          *vit *= psd_vector[RBnumber];
          ++vit;
          ++fit;
        }

    }
//  std::cout<< "Exiting spectrum model..."<< std::endl;
  return rxPsd;
}

void
SivertSpectrumPropagationLossModel::SetChannelFromUnity3D(int TxID, int RxID, std::vector<double> gainVector)
{

  m_Channel_from_Unity3D[TxID][RxID] = gainVector;
  m_Channel_from_Unity3D[RxID][TxID] = gainVector;

}





}  // namespace ns3
