#include "ns3/core-module.h"
#include "ns3/propagation-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/wifi-module.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <vector>
NS_LOG_COMPONENT_DEFINE ("Main");

using namespace ns3;

#define Downlink true
#define Uplink false
#define PI 3.14159265
#define PI_e5 314158

class Experiment
{
public:
  Experiment(bool downlinkUplink, std::string in_modes);
  void SetRtsCts(bool enableCtsRts);
  void CreateNode(size_t in_ap, size_t in_nodeNumber, double radius);
  void CreateNode(size_t in_ap, size_t in_nodeNumber);
  void InitialExperiment();
  void InstallApplication(size_t in_packetSize, size_t in_dataRate);
  void Run(size_t in_simTime);
  void PhyRxErrorTrace (std::string context, Ptr<const Packet> packet, double snr);   
  void PhyRxOkTrace (std::string context, Ptr<const Packet> packet, 
                     double snr, WifiMode mode, enum WifiPreamble preamble);
  void PhyTxTrace (std::string context, Ptr<const Packet> packet, WifiMode mode, 
              WifiPreamble preamble, uint8_t txPower);
  void ShowNodeInformation(NodeContainer in_c, size_t in_numOfNode);

private:  
  void SetWifiChannel();
  void InstallDevices();  
  void InstallIp();
  
  bool m_enableCtsRts;
  bool m_downlinkUplink;
  size_t m_apNumber;
  size_t m_nodeNumber;
  double m_radius;
  size_t m_rxOkCount;
  size_t m_rxErrorCount;
  size_t m_txOkCount;
  std::string m_modes;
  std::vector<std::vector<double> > m_readChannelGain;
  std::vector<int> m_serveBy;
  NodeContainer m_nodes;
  MobilityHelper m_mobility;
  Ptr<ListPositionAllocator> m_apPosAlloc;
  Ptr<ListPositionAllocator> m_nodePosAlloc;
  YansWifiChannelHelper m_wifiChannel;
  WifiHelper m_wifi;
  YansWifiPhyHelper m_wifiPhy;
  NqosWifiMacHelper m_wifiMac;
  NetDeviceContainer m_devices;
  InternetStackHelper m_internet;
  Ipv4AddressHelper m_ipv4;
  ApplicationContainer m_cbrApps;
  ApplicationContainer m_pingApps;
};

Experiment::Experiment(bool in_downlinkUplink, std::string in_modes):
  m_downlinkUplink(in_downlinkUplink), m_modes(in_modes)
{
  m_rxOkCount = 0;
  m_rxErrorCount = 0;
  m_txOkCount = 0;
}

void
Experiment::InitialExperiment()
{
  SetWifiChannel();
  InstallDevices();  
  InstallIp();
}

void
Experiment::SetRtsCts(bool in_enableCtsRts)
{
  m_enableCtsRts = in_enableCtsRts;
  UintegerValue ctsThr = (m_enableCtsRts ? UintegerValue (10) : UintegerValue (22000));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);
}


void
Experiment::CreateNode(size_t in_ap, size_t in_nodeNumber, double in_radius)
{  
  m_apNumber = in_ap;
  m_nodeNumber = in_nodeNumber;
  m_radius = in_radius; 
  
  m_nodes.Create(m_apNumber+m_nodeNumber);
  
  m_mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  m_mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  m_apPosAlloc = CreateObject<ListPositionAllocator> ();
  m_nodePosAlloc = CreateObject<ListPositionAllocator> ();
  
  for(size_t i=0; i<m_apNumber; ++i){
    m_apPosAlloc->Add(Vector(m_radius*std::cos(i*2*PI/m_apNumber), 
                      m_radius*std::sin(i*2*PI/m_apNumber), 1));
    //m_apPosAlloc->Add(Vector(m_radius, 0, 1));
  }
  m_mobility.SetPositionAllocator(m_apPosAlloc);
  for(size_t i=0; i<m_apNumber; ++i){
    m_mobility.Install(m_nodes.Get(i));  
  }
  
  for(size_t i=0; i<m_nodeNumber; ++i){
   size_t inAp = i/(m_nodeNumber/m_apNumber);
   double nodeRadius = rand()%120+(rand()%1000)/1000;
   m_nodePosAlloc->Add(Vector(m_radius*std::cos(inAp*2*PI/m_apNumber)+
                       nodeRadius*std::cos((rand()%(2*PI_e5))/pow(10, 5)), 
                       m_radius*std::sin(inAp*2*PI/m_apNumber)+
                       nodeRadius*std::sin((rand()%(2*PI_e5))/pow(10, 5)), 
                       1));
   //m_nodePosAlloc->Add(Vector(0, 0, 1));
  }
  m_mobility.SetPositionAllocator(m_nodePosAlloc);
  for(size_t i=0; i<m_nodeNumber; ++i){
    m_mobility.Install(m_nodes.Get(m_apNumber+i));  
  }   
}

void
Experiment::SetWifiChannel()
{
  m_wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  m_wifiChannel.AddPropagationLoss ("ns3::TwoRayGroundPropagationLossModel",
                                  "Frequency", DoubleValue(2.400e9));
}

void
Experiment::InstallDevices()
{
  m_wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  //Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
  //                            StringValue ("DsssRate2Mbps"));
  m_wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", 
                                "DataMode",StringValue (m_modes), 
                                "ControlMode",StringValue (m_modes));
                                
                                
  m_wifiPhy =  YansWifiPhyHelper::Default ();
  m_wifiPhy.SetChannel (m_wifiChannel.Create());
  m_wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-95.0) );
  m_wifiPhy.Set ("CcaMode1Threshold", DoubleValue (-95.0) );
  m_wifiPhy.Set ("TxPowerStart", DoubleValue (23.0) );
  m_wifiPhy.Set ("TxPowerEnd", DoubleValue (23.0) );
  m_wifiPhy.Set ("ChannelNumber", UintegerValue (1) );
  m_wifiPhy.Set ("RxGain", DoubleValue (-25.0));    
  NqosWifiMacHelper m_wifiMac;
  m_wifiMac.SetType ("ns3::AdhocWifiMac"); // use ad-hoc MAC
  m_devices = m_wifi.Install (m_wifiPhy, m_wifiMac, m_nodes); 
}

void
Experiment::InstallIp()
{  
  m_internet.Install (m_nodes);  
  m_ipv4.SetBase ("10.0.0.0", "255.0.0.0");
  m_ipv4.Assign (m_devices); 
}

void
Experiment::PhyRxErrorTrace (std::string context, Ptr<const Packet> packet, double snr)
{
  Ptr<Packet> m_currentPacket;
  WifiMacHeader hdr;
  m_currentPacket = packet->Copy();
  m_currentPacket->RemoveHeader (hdr);
  if(hdr.IsData()){
    m_rxErrorCount++;
  }
}

void
Experiment::PhyRxOkTrace (std::string context, Ptr<const Packet> packet, 
        double snr, WifiMode mode, enum WifiPreamble preamble)
{
  Ptr<Packet> m_currentPacket;
  WifiMacHeader hdr;
  
  m_currentPacket = packet->Copy();
  m_currentPacket->RemoveHeader (hdr);  
  if(hdr.IsData()){    
    m_rxOkCount++;
  }
}

void
Experiment::PhyTxTrace (std::string context, Ptr<const Packet> packet, 
                  WifiMode mode, WifiPreamble preamble, uint8_t txPower)
{
  Ptr<Packet> m_currentPacket;
  WifiMacHeader hdr;
  m_currentPacket = packet->Copy();
  m_currentPacket->RemoveHeader (hdr);
  if(hdr.IsData()){
    m_txOkCount++;
  }
}



void
Experiment::InstallApplication(size_t in_packetSize, size_t in_dataRate)
{
  uint16_t cbrPort = 12345;
  for(size_t j=1; j<=m_apNumber; ++j){
    for(size_t i=m_apNumber+m_nodeNumber/m_apNumber*(j-1); 
        i<m_apNumber+m_nodeNumber/m_apNumber*j ; ++i){
      std::string s;
      std::stringstream ss(s);
      if(m_downlinkUplink){
         ss << i+1;
      }else
      {
        ss << j;
      }
      s = "10.0.0."+ss.str();
      OnOffHelper onOffHelper ("ns3::UdpSocketFactory", 
               InetSocketAddress (Ipv4Address (s.c_str()), cbrPort));
      onOffHelper.SetAttribute ("PacketSize", UintegerValue (in_packetSize));
//onOffHelper.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
//onOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
      std::string s2;
      std::stringstream ss2(s2);
      if(m_downlinkUplink){
        ss2 << in_dataRate+i*100;
      }else
      {
        ss2 << in_dataRate+i*100;
      }
      s2 = ss2.str() + "bps";
      onOffHelper.SetAttribute ("DataRate", StringValue (s2));
      if(m_downlinkUplink){
        onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (1.00+static_cast<double>(i)/100)));
        onOffHelper.SetAttribute ("StopTime", TimeValue (Seconds (50.000+static_cast<double>(i)/100)));
        m_cbrApps.Add (onOffHelper.Install (m_nodes.Get (j-1)));
      }else
      {
        onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (1.00)));
        onOffHelper.SetAttribute ("StopTime", TimeValue (Seconds (50.000+static_cast<double>(j)/100)));
        m_cbrApps.Add (onOffHelper.Install (m_nodes.Get (i))); 
      }
    }
  }  
  uint16_t  echoPort = 9;   
  // again using different start times to workaround Bug 388 and Bug 912
  for(size_t j=1; j<=m_apNumber; ++j){
    for(size_t i=m_apNumber+m_nodeNumber/m_apNumber*(j-1); 
        i<m_apNumber+m_nodeNumber/m_apNumber*j ; ++i){
      std::string s;
      std::stringstream ss(s);
      if(m_downlinkUplink){
         ss << i+1;
      }else
      {
        ss << j;
      }
      s = "10.0.0."+ss.str();
      UdpEchoClientHelper echoClientHelper (Ipv4Address (s.c_str()), echoPort);
      echoClientHelper.SetAttribute ("MaxPackets", UintegerValue (1));
      echoClientHelper.SetAttribute ("Interval", TimeValue (Seconds (0.1)));
      echoClientHelper.SetAttribute ("PacketSize", UintegerValue (10));
      if(m_downlinkUplink){
        echoClientHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001)));
        echoClientHelper.SetAttribute ("StopTime", TimeValue (Seconds (50.001)));
        m_pingApps.Add (echoClientHelper.Install (m_nodes.Get (j-1))); 
      }else
      {
        echoClientHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001)));
        echoClientHelper.SetAttribute ("StopTime", TimeValue (Seconds (50.001)));
        m_pingApps.Add (echoClientHelper.Install (m_nodes.Get (i))); 
      }
    }
  }
}


void
Experiment::ShowNodeInformation(NodeContainer in_c, size_t in_numOfNode)
{
  for(size_t i=0; i<in_numOfNode; ++i){
    Ptr<MobilityModel> mobility = in_c.Get(i)->GetObject<MobilityModel> ();
    Vector nodePos = mobility->GetPosition ();
    // Get Ipv4 instance of the node
    Ptr<Ipv4> ipv4 = in_c.Get(i)->GetObject<Ipv4> (); 
    // Get Ipv4 instance of the node
    Ptr<MacLow> mac48 = in_c.Get(i)->GetObject<MacLow> (); 
    // Get Ipv4InterfaceAddress of xth interface.
    Ipv4Address addr = ipv4->GetAddress (1, 0).GetLocal ();     
    //Mac48Address macAddr = mac48->GetAddress();
    std::cout << in_c.Get(i)->GetId() << " " << addr << " (" << nodePos.x << ", " <<  
               nodePos.y << ")" << std::endl;
  }
}

void 
Experiment::Run(size_t in_simTime)
{ 
  // 8. Install FlowMonitor on all nodes
  ShowNodeInformation(m_nodes, m_apNumber+m_nodeNumber);
  
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  // 9. Run simulation
  Simulator::Stop (Seconds (in_simTime));
  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/RxError", 
             MakeCallback (&Experiment::PhyRxErrorTrace, this));
  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/RxOk", 
             MakeCallback (&Experiment::PhyRxOkTrace, this));
  Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/Tx", 
             MakeCallback (&Experiment::PhyTxTrace, this));
  Simulator::Run ();

  // 10. Print per flow statistics
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  double accumulatedThroughput = 0;
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i=stats.begin(); 
        i!=stats.end(); ++i)
  {    
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
    std::cout << "Flow " << i->first<< " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
    std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
    std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
    std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
    std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
    std::cout << "  Lost Packets: " << i->second.lostPackets << "\n";
    std::cout << "  Pkt Lost Ratio: " << ((double)i->second.txPackets-(double)i->second.rxPackets)/(double)i->second.txPackets << "\n";
    std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / in_simTime / 1024 / 1024  << " Mbps\n";
    accumulatedThroughput+=(i->second.rxBytes*8.0/in_simTime/1024/1024);
  }  
  std::cout << "apNumber=" <<m_apNumber << " nodeNumber=" << m_nodeNumber << "\n" << std::flush;
  std::cout << "throughput=" << accumulatedThroughput << "\n" << std::flush;
  std::cout << "tx=" << m_txOkCount << " RXerror=" <<m_rxErrorCount << 
               " Rxok=" << m_rxOkCount << "\n" << std::flush;
  std::cout << "===========================\n" << std::flush;
  // 11. Cleanup
  Simulator::Destroy ();
}

int main (int argc, char **argv)
{ 
  
  size_t numOfAp[6] = {1, 2, 3, 4, 5, 6};
  double range[4] = {60, 120, 180, 240};
  std::vector <std::string> modes;
  modes.push_back ("DsssRate1Mbps");
  modes.push_back ("DsssRate2Mbps");
  modes.push_back ("DsssRate5_5Mbps");
  modes.push_back ("DsssRate11Mbps");
  std::cout << "Hidden station experiment with RTS/CTS disabled:\n" << std::flush;
  for(size_t i=0; i<1; ++i){
    for(size_t j=0; j<1; ++j){
      for(size_t k=2; k<3; ++k){
        std::cout << "Range=" << range[j] << ", Mode=" << modes[k] << "\n";
        Experiment exp(Downlink, modes[k]);
        exp.SetRtsCts(false);
        exp.CreateNode(numOfAp[i], 2, range[j]);
        exp.InitialExperiment();
        exp.InstallApplication(1024, 5500000);
        exp.Run(60);
      }
    }
  }
  /*
  std::cout << "Hidden station experiment with RTS/CTS enable:\n" << std::flush;
  for(size_t i=0; i<6; ++i){
    for(size_t j=0; j<4; ++j){
      for(size_t k=0; k<modes.size(); ++k){
        std::cout << "Range=" << range[j] << "Mode=" << modes[k] << "\n";
        Experiment exp(Downlink, modes[k]);
        exp.SetRtsCts(true);
        exp.CreateNode(numOfAp[i], 60, range[j]);
        exp.InitialExperiment();
        exp.InstallApplication(1024, 16000000);
        exp.Run(60);
      }
    }
  }
  */
  return 0;
}
