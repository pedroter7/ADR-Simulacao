/*
  This simulation was modified for a project for the course
  Analise e Desempenho de Redes, UFABC - 2022 Q1

  The modifications extend the default script mainly by 
  implementing multiple ping sources, end-to-end delay 
  calculation and packet interval based in a random variable
*/

/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008,2009 IITP RAS
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
 * Author: Kirill Andreev <andreev@iitp.ru>
 *
 *
 * By default this script creates m_xSize * m_ySize square grid topology with
 * IEEE802.11s stack installed at each node with peering management
 * and HWMP protocol.
 * The side of the square cell is defined by m_step parameter.
 * When topology is created, UDP ping is installed to opposite corners
 * by diagonals. packet size of the UDP ping and interval between two
 * successive packets is configurable.
 * 
 *  m_xSize * step
 *  |<--------->|
 *   step
 *  |<--->|
 *  * --- * --- * <---Ping sink  _
 *  | \   |   / |                ^
 *  |   \ | /   |                |
 *  * --- * --- * m_ySize * step |
 *  |   / | \   |                |
 *  | /   |   \ |                |
 *  * --- * --- *                _
 *  ^ Ping source
 *
 * By varying m_xSize and m_ySize, one can configure the route that is used.
 * When the inter-nodal distance is small, the source can reach the sink
 * directly.  When the inter-nodal distance is intermediate, the route
 * selected is diagonal (two hop).  When the inter-nodal distance is a bit
 * larger, the diagonals cannot be used and a four-hop route is selected.
 * When the distance is a bit larger, the packets will fail to reach even the
 * adjacent nodes.
 *
 * As of ns-3.36 release, with default configuration (mesh uses Wi-Fi 802.11a
 * standard and the ArfWifiManager rate control by default), the maximum
 * range is roughly 50m.  The default step size in this program is set to 50m,
 * so any mesh packets in the above diagram depiction will not be received
 * successfully on the diagonal hops between two nodes but only on the
 * horizontal and vertical hops.  If the step size is reduced to 35m, then
 * the shortest path will be on the diagonal hops.  If the step size is reduced
 * to 17m or less, then the source will be able to reach the sink directly
 * without any mesh hops (for the default 3x3 mesh depicted above). 
 *
 * The position allocator will lay out the nodes in the following order
 * (corresponding to Node ID and to the diagram above):
 *
 * 6 - 7 - 8
 * |   |   | 
 * 3 - 4 - 5
 * |   |   | 
 * 0 - 1 - 2
 *
 *  See also MeshTest::Configure to read more about configurable
 *  parameters.
 */

#include <iostream>
#include <sstream>
#include <fstream>
#include <map>
#include <vector>
#include <mutex>
#include <algorithm>
#include <numeric>
#include <time.h>
#include <iomanip>
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/aodv-helper.h"
#include "ns3/olsr-helper.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"
 
using namespace ns3;
 
NS_LOG_COMPONENT_DEFINE ("MeshExample");
 
// Declaring these variables outside of main() for use in trace sinks
uint32_t g_udpTxCount = 0;
uint32_t g_udpRxCount = 0;
uint64_t g_bytesRxCount = 0;
std::map<uint64_t, Time> g_packageSentTimeMap;
std::vector<uint64_t> g_endToEndDelays;
std::mutex g_mutex;
 
void
TxTrace (Ptr<const Packet> p)
{
  NS_LOG_DEBUG ("Sent " << p->GetSize () << " bytes"); 
  g_mutex.lock(); 
  g_udpTxCount++;
  g_packageSentTimeMap.insert(std::pair<uint64_t, Time>(p->GetUid(), Simulator::Now()));
  g_mutex.unlock();
}
 
void
RxTrace (Ptr<const Packet> p)
{
  NS_LOG_DEBUG ("Received " << p->GetSize () << " bytes");  
  g_mutex.lock();
  g_udpRxCount++;
  g_bytesRxCount += p->GetSize();
  // End-To-End Delay
  Time difference = Simulator::Now() - g_packageSentTimeMap.at(p->GetUid());
  g_endToEndDelays.push_back(difference.ToInteger(Time::Unit::MS));
  g_mutex.unlock();
}
 
enum RoutingProtocolEnum
{
  AODV,
  OLSR
};

typedef struct SimResults
{
  uint64_t endToEndDelay;
  uint64_t throughputBytes;
  uint32_t throughputPackets;
} SimResults;
 
class MeshTest
{
public:
  MeshTest ();
  void Configure (int argc, char ** argv);
  int Run ();
private:
  int       m_xSize; 
  int       m_ySize; 
  double    m_step; 
  double    m_randomStart; 
  double    m_totalTime; 
  uint16_t  m_packetSize; 
  uint32_t  m_nIfaces; 
  bool      m_chan; 
  bool      m_pcap; 
  bool      m_ascii; 
  std::string m_stack; 
  std::string m_root; 
  NodeContainer nodes;
  NetDeviceContainer meshDevices;
  Ipv4InterfaceContainer interfaces;
  MeshHelper mesh;
  RoutingProtocolEnum m_routingProtocol;
  std::string m_outputDir;
  std::string m_routingProtocolName;
  std::string m_fullOutFilePath;
  uint16_t m_nClients;
private:
  void CreateNodes ();
  void InstallInternetStack ();
  void InstallApplication ();
  void Report ();
  void WriteResults (std::ostream& os, SimResults results);
  SimResults CalculateResults ();
};
MeshTest::MeshTest () :
  m_xSize (3),
  m_ySize (3),
  m_step (50.0),
  m_randomStart (0.1),
  m_totalTime (100.0),
  m_packetSize (1024),
  m_nIfaces (1),
  m_chan (true),
  m_pcap (false),
  m_ascii (false),
  m_stack ("ns3::Dot11sStack"),
  m_root ("ff:ff:ff:ff:ff:ff"),
  m_routingProtocol(RoutingProtocolEnum::AODV),
  m_outputDir(""),
  m_routingProtocolName ("AODV"),
  m_fullOutFilePath(""),
  m_nClients(1)
{
}
void
MeshTest::Configure (int argc, char *argv[])
{
  CommandLine cmd (__FILE__);
  cmd.AddValue ("x-size", "Number of nodes in a row grid", m_xSize);
  cmd.AddValue ("y-size", "Number of rows in a grid", m_ySize);
  cmd.AddValue ("step",   "Size of edge in our grid (meters)", m_step);
  // Avoid starting all mesh nodes at the same time (beacons may collide)
  cmd.AddValue ("start",  "Maximum random start delay for beacon jitter (sec)", m_randomStart);
  cmd.AddValue ("time",  "Simulation time (sec)", m_totalTime);
  cmd.AddValue ("packet-size",  "Size of packets in UDP ping (bytes)", m_packetSize);
  cmd.AddValue ("interfaces", "Number of radio interfaces used by each mesh point", m_nIfaces);
  cmd.AddValue ("channels",   "Use different frequency channels for different interfaces", m_chan);
  cmd.AddValue ("pcap",   "Enable PCAP traces on interfaces", m_pcap);
  cmd.AddValue ("ascii",   "Enable Ascii traces on interfaces", m_ascii);
  cmd.AddValue ("stack",  "Type of protocol stack. ns3::Dot11sStack by default", m_stack);
  cmd.AddValue ("root", "Mac address of root mesh point in HWMP", m_root);
  ushort protocol = 1;
  cmd.AddValue ("routing-protocol", "Routing protocol: 1 - ADOV; 2 - OLSR; [AODV default]", protocol);
  cmd.AddValue ("out-dir", "Output dir to write the results file", m_outputDir);
  cmd.AddValue ("clients", "Number of client nodes", m_nClients);
 
  cmd.Parse (argc, argv);
  // Set routing protocol
  if (protocol == 1) 
  {
    std::cout << "Using AODV" << std::endl;
    m_routingProtocol = RoutingProtocolEnum::AODV;
    m_routingProtocolName = "AODV";
  }
  else
  {
    std::cout << "Using OLSR" << std::endl;
    m_routingProtocol = RoutingProtocolEnum::OLSR;
    m_routingProtocolName = "OLSR";
  }

  // Configure output file path
  std::stringstream s;
  s << m_outputDir << "/" << "SimulationResults_RoutingProtocol_"
    << m_routingProtocolName << "_NodeNumber_" << m_xSize*m_ySize
    << ".txt";

  m_fullOutFilePath = s.str();

  NS_LOG_DEBUG ("Grid:" << m_xSize << "*" << m_ySize);
  NS_LOG_DEBUG ("Simulation time: " << m_totalTime << " s");
  if (m_ascii)
    {
      PacketMetadata::Enable ();
    }
}
void
MeshTest::CreateNodes ()
{ 
  /*
   * Create m_ySize*m_xSize stations to form a grid topology
   */
  nodes.Create (m_ySize*m_xSize);
  // Configure YansWifiChannel
  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  /*
   * Create mesh helper and set stack installer to it
   * Stack installer creates all needed protocols and install them to
   * mesh point device
   */
  mesh = MeshHelper::Default ();
  if (!Mac48Address (m_root.c_str ()).IsBroadcast ())
    {
      mesh.SetStackInstaller (m_stack, "Root", Mac48AddressValue (Mac48Address (m_root.c_str ())));
    }
  else
    {
      //If root is not set, we do not use "Root" attribute, because it
      //is specified only for 11s
      mesh.SetStackInstaller (m_stack);
    }
  if (m_chan)
    {
      mesh.SetSpreadInterfaceChannels (MeshHelper::SPREAD_CHANNELS);
    }
  else
    {
      mesh.SetSpreadInterfaceChannels (MeshHelper::ZERO_CHANNEL);
    }
  mesh.SetMacType ("RandomStart", TimeValue (Seconds (m_randomStart)));
  // Set number of interfaces - default is single-interface mesh point
  mesh.SetNumberOfInterfaces (m_nIfaces);
  // Install protocols and return container if MeshPointDevices
  meshDevices = mesh.Install (wifiPhy, nodes);
  // AssignStreams can optionally be used to control random variable streams
  mesh.AssignStreams (meshDevices, 0);
  // Setup mobility - static grid topology
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (m_step),
                                 "DeltaY", DoubleValue (m_step),
                                 "GridWidth", UintegerValue (m_xSize),
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);
  if (m_pcap)
    wifiPhy.EnablePcapAll (std::string ("mp"));
  if (m_ascii)
    {
      AsciiTraceHelper ascii;
      wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("mesh.tr"));
    }
}
void
MeshTest::InstallInternetStack ()
{
  InternetStackHelper internetStack;
  // Config routing protocol
  Ipv4ListRoutingHelper routingHelper;
  AodvHelper aodv;
  OlsrHelper olsr;
  if (m_routingProtocol == RoutingProtocolEnum::AODV)
  {
    routingHelper.Add(aodv, 100);
  }
  else
  {
    routingHelper.Add(olsr, 100);
  }
  internetStack.SetRoutingHelper(routingHelper);
  internetStack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.0.0.0");
  interfaces = address.Assign (meshDevices);
}
void
MeshTest::InstallApplication ()
{
  uint16_t portNumber = 9;
  UdpEchoServerHelper echoServer (portNumber);
  uint16_t sinkNodeId = m_xSize * m_ySize - 1;
  ApplicationContainer serverApps = echoServer.Install (nodes.Get (sinkNodeId));
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (m_totalTime + 1));
  UdpEchoClientHelper echoClient (interfaces.GetAddress (sinkNodeId), portNumber);
  echoClient.SetAttribute ("PacketSize", UintegerValue (m_packetSize));

  std::vector<uint32_t> installedNodes;
  Ptr<UniformRandomVariable> randVariable = CreateObject<UniformRandomVariable>();
  uint32_t randomNode = 0;
  for (size_t i = 0; i < m_nClients; ++i)
  {
    double packetInterval = randVariable->GetValue(0.01, 1.0);
    echoClient.SetAttribute ("MaxPackets", UintegerValue ((uint32_t)(m_totalTime*(1/packetInterval))));
    echoClient.SetAttribute ("Interval", TimeValue (Seconds (packetInterval)));
    if (i > 0)
    {
      // Get random node
      do
      {
        randomNode = randVariable->GetInteger(1, sinkNodeId-1);
      } while (std::find(installedNodes.begin(), installedNodes.end(), randomNode) != installedNodes.end());
    }
    ApplicationContainer clientApps = echoClient.Install (nodes.Get (randomNode));
    Ptr<UdpEchoClient> app = clientApps.Get (0)->GetObject<UdpEchoClient> ();
    app->TraceConnectWithoutContext ("Tx", MakeCallback (&TxTrace));
    app->TraceConnectWithoutContext ("Rx", MakeCallback (&RxTrace));
    clientApps.Start (Seconds (1.0));
    clientApps.Stop (Seconds (m_totalTime + 1.5));
    installedNodes.push_back(randomNode);
    std::cout << "Client installed in node" << randomNode << std::endl;
  }
}
int
MeshTest::Run ()
{
  CreateNodes ();
  InstallInternetStack ();
  InstallApplication ();
  Simulator::Schedule (Seconds (m_totalTime), &MeshTest::Report, this);
  Simulator::Stop (Seconds (m_totalTime + 2));
  std::cout << "Running simulation..." << std::endl;
  Simulator::Run ();
  Simulator::Destroy ();
  std::cout << "UDP echo packets sent: " << g_udpTxCount << " received: " << g_udpRxCount << std::endl;
  return 0;
}
void
MeshTest::Report ()
{
  std::ofstream of;
  of.open(m_fullOutFilePath, std::ios::app);
  if (!of.is_open())
  {
    std::cout << "Could not open file for writing. File path was: " 
              << m_fullOutFilePath << std::endl << "Results will be written to stdout"
              << std::endl;
    
    WriteResults(std::cout, CalculateResults());
  }
  else
  {
    std::cout << "Writing simulation results to " << m_fullOutFilePath << std::endl;
    WriteResults(of, CalculateResults());
  }
}
void
MeshTest::WriteResults (std::ostream& os, SimResults results)
{
  time_t t = time(NULL);

  os << std::endl << "---Simulation Results Begin---" << std::endl
    << "Execution time: " << ctime(&t) << std::endl
    << "Nodes: " << m_xSize*m_ySize << std::endl
    << "Clients (ping sources): " << m_nClients << std::endl
    << "Space between nodes (m): " << m_step << std::endl
    << "Routing protocol: " << m_routingProtocolName << std::endl
    << "Packet size (bytes): " << m_packetSize << std::endl
    << "Simulation Total Time (s): " << m_totalTime << std::endl
    << "Average End-To-End delay (ms): " << results.endToEndDelay << std::endl
    << "Throughput (packets/s): " << results.throughputPackets << std::endl
    << "Throughput (bytes/s): " << results.throughputBytes << std::endl
    << "Packets sent: " << g_udpTxCount << std::endl
    << "Packets received: " << g_udpRxCount << std::endl
    << "---Simulation Results End---" << std::endl;
}
SimResults
MeshTest::CalculateResults ()
{
  SimResults results;
  g_mutex.lock();
  results.endToEndDelay = (uint64_t) std::accumulate(g_endToEndDelays.begin(), g_endToEndDelays.end(), 0)/g_endToEndDelays.size();
  results.throughputBytes = (uint64_t) g_bytesRxCount / m_totalTime;
  results.throughputPackets = (uint32_t) g_udpRxCount / m_totalTime;
  g_mutex.unlock();
  return results;
}
int
main (int argc, char *argv[])
{
  MeshTest t; 
  t.Configure (argc, argv);
  return t.Run ();
}