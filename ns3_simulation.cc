/*
    Simulacao de uma rede mesh utilizando o simulador ns3.
    Essa simulacao eh parte do trabalho da disciplina
    Avaliacao de Desempenho de Redes da UFABC
    Prof. Carlo Kleber, Q1-2022

    Essa simulacao utiliza como base pedacos de codigo das seguintes fontes:

    mesh network example: https://www.nsnam.org/doxygen/mesh_8cc_source.html
    manet routing compare example: https://www.nsnam.org/doxygen/manet-routing-compare_8cc_source.html
*/

#include <string>
#include <sstream>
#include <ns3/core-module.h>
#include <ns3/node-container.h>
#include <ns3/wimax-helper.h>
#include <ns3/net-device-container.h>
#include <ns3/ipv4-address-helper.h>
#include <ns3/mobility-helper.h>
#include <ns3/udp-client-server-helper.h>
#include <ns3/internet-stack-helper.h>
#include <ns3/olsr-helper.h>
#include <ns3/aodv-helper.h>
#include <ns3/ipv4-list-routing-helper.h>
#include <ns3/vector.h>
#include <ns3/random-variable-stream.h>
#include <cmath>

#define EXPERIMENT_NAME "Experimento_ADR"
#define PROTOCOLO_AODV 1
#define PROTOCOLO_OLSR 2
#define PROTOCOLO_AODV_NOME "AODV"
#define PROTOCOLO_OLSR_NOME "OLSR"
#define DEFAULT_LARGURA_ROTAS 20.0 // km
#define DEFAULT_DISTANCIA_ESTACAO_TERRESTRE 10.0 // km
#define DEFAULT_DISTANCIA_MINIMA_ENTRE_NAVIOS 5.0 // km
#define DEFAULT_COMPRIMENTO_ROTA 60.0 // km
#define UDP_SERVER_PORT 300

using namespace ns3;

enum RotaDirecao
{
	LESTE,
	OESTE
};

NS_LOG_COMPONENT_DEFINE(EXPERIMENT_NAME);

class Experiment
{
public:
	Experiment();
	void CommandSetup(int argc, char **argv);
	void Run();
private:
	uint32_t m_protocolo;
	std::string m_nomeProtocolo;
	std::string m_outDir;
	double m_distanciaEstacaoTerrestre;
	double m_larguraRotas;
	double m_comprimentoRota;
	double m_tempoSimulacao;
	double m_distanciaMinimaEntreNavios;
	NodeContainer m_estacaoTeNoderrestreNodeContainer;
	NodeContainer m_rotaOesteNodeContainer;
	NodeContainer m_rotaLesteNodeContainer;
	Ipv4AddressHelper m_ipv4AdressHelper;
	Ipv4InterfaceContainer m_ipInterfaceContainerEstacaoTerrestre;
	InternetStackHelper m_internetHelper;
	// Controle para nao inserir um navio em cima de outro
	double m_ultimoNavioRotaLesteY;
	double m_ultimoNavioRotaOesteY;

	void ConfigurarSimulacao();
	void SetNomeProtocolo();
	void ConfigurarEstacaoTerrestre();
	void ConfigurarInternetStack();
	void ConfigurarLog();
	void InserirNavioNaRota(RotaDirecao rota, double velocidade);
	Ptr<WimaxChannel> CriaWimaxChannel();
	double CalculaYNovoNavio(RotaDirecao rota);
};

Experiment::Experiment() 
    :   m_protocolo(PROTOCOLO_AODV),
        m_nomeProtocolo(PROTOCOLO_AODV_NOME),
		m_larguraRotas(DEFAULT_LARGURA_ROTAS),
		m_distanciaEstacaoTerrestre(DEFAULT_DISTANCIA_ESTACAO_TERRESTRE),
		m_comprimentoRota(DEFAULT_COMPRIMENTO_ROTA),
		m_distanciaMinimaEntreNavios(DEFAULT_DISTANCIA_MINIMA_ENTRE_NAVIOS),
		m_ipv4AdressHelper(),
		m_rotaLesteNodeContainer(),
		m_rotaOesteNodeContainer(),
		m_internetHelper(),
		m_ultimoNavioRotaLesteY(-1.0),
		m_ultimoNavioRotaOesteY(-1.0)
{
	m_ipv4AdressHelper.SetBase("10.1.0.0", "255.255.0.0");
}

void Experiment::CommandSetup(int argc, char **argv)
{
	CommandLine cmd(__FILE__);
	cmd.AddValue("output-dir", "Diretorio para escrever os resultados da simulacao", m_outDir);
	cmd.AddValue("protocol", "1=AODV;2=OLSR", m_protocolo);
	cmd.AddValue("tempo-simulacao", "Tempo de simulacao", m_tempoSimulacao);
	cmd.AddValue("distancia-ls", "Distancia da estacao terrestre ate a rota mais proxima", m_distanciaEstacaoTerrestre);
	cmd.AddValue("comprimento-rotas", "Comprimento das rotas, i.e. comprimento da area de trafego", m_comprimentoRota);
	cmd.AddValue("largura-rotas", "Largura das rotas", m_larguraRotas);
	cmd.AddValue("distancia-navios", "Distancia minima entre navios nas rotas", m_distanciaMinimaEntreNavios);
	cmd.Parse(argc, argv);
}

void Experiment::Run()
{
	ConfigurarSimulacao();
	// TODO iniciar simulacao
	// TODO rotina para instanciar navios nas rotas
	// TODO rotina para deletar nodes de navios que sairam da rota
	// TODO rotina para coletar dados da simulacao
	// TODO Parar simulacao
	// TODO Escrever resultados
	// TODO Limpar memoria	
}

void Experiment::SetNomeProtocolo()
{
	switch (m_protocolo)
	{
		case PROTOCOLO_AODV:
			m_nomeProtocolo = PROTOCOLO_AODV_NOME;
			break;
		case PROTOCOLO_OLSR:
			m_nomeProtocolo = PROTOCOLO_OLSR_NOME;
			break;
		default:
			NS_FATAL_ERROR("Protocolo desconhecido: " << m_protocolo);
	}

	NS_LOG_INFO("Protocolo da simulacao: " << m_nomeProtocolo);
}

void Experiment::ConfigurarSimulacao()
{
	ConfigurarLog();
	ConfigurarInternetStack();
	ConfigurarEstacaoTerrestre();
}

void Experiment::ConfigurarLog()
{
	LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
	LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
}

void Experiment::ConfigurarEstacaoTerrestre()
{
	m_estacaoTeNoderrestreNodeContainer.Create(1);

	WimaxHelper wimaxHelper;
	NetDeviceContainer netDevContainer;
	netDevContainer = wimaxHelper.Install(m_estacaoTeNoderrestreNodeContainer,
											WimaxHelper::DEVICE_TYPE_BASE_STATION,
											WimaxHelper::SIMPLE_PHY_TYPE_OFDM,
											CriaWimaxChannel(),
											WimaxHelper::SCHED_TYPE_SIMPLE);
	wimaxHelper.EnableAscii("land-station", m_estacaoTeNoderrestreNodeContainer);
	
	m_ipInterfaceContainerEstacaoTerrestre = m_ipv4AdressHelper.Assign(netDevContainer);

	// Posiciona a estacao 
	MobilityHelper mHelper;
	mHelper.SetPositionAllocator("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue(m_comprimentoRota/2),
                                 "MinY", DoubleValue(0.0));
	mHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mHelper.Install(m_estacaoTeNoderrestreNodeContainer);

	// Configura a aplicacao servidor udp
	UdpServerHelper serverHelper(UDP_SERVER_PORT);
	ApplicationContainer estacaoApplicationContainer;
	estacaoApplicationContainer = serverHelper.Install(m_estacaoTeNoderrestreNodeContainer);
	estacaoApplicationContainer.Start(Seconds(1));
	estacaoApplicationContainer.Stop(Seconds(m_tempoSimulacao + 1.0));

	// Instala o internet stack
	m_internetHelper.Install(m_estacaoTeNoderrestreNodeContainer);
}

void Experiment::ConfigurarInternetStack()
{
	Ipv4ListRoutingHelper routingHelper;
	AodvHelper aodvHelper;
	OlsrHelper olsrHelper;
	switch (m_protocolo)
	{
	case PROTOCOLO_AODV:
		routingHelper.Add(aodvHelper, 100);
		break;

	case PROTOCOLO_OLSR:
		routingHelper.Add(olsrHelper, 100);
		break;
	
	default:
		NS_FATAL_ERROR("Protocol not found: " << m_protocolo);
	}

	m_internetHelper.SetRoutingHelper(routingHelper);
}

void Experiment::InserirNavioNaRota(RotaDirecao rota, double velocidade)
{
	NodeContainer container;
	container.Create(1);

	// Configura Wimax
	WimaxHelper wimaxHelper;
	NetDeviceContainer netDevContainer;
	netDevContainer = wimaxHelper.Install(container,
											WimaxHelper::DEVICE_TYPE_SUBSCRIBER_STATION,
											WimaxHelper::SIMPLE_PHY_TYPE_OFDM,
											CriaWimaxChannel(),
											WimaxHelper::SCHED_TYPE_SIMPLE);

	std::stringstream strStream;
	strStream << "navio" << container.Get(0)->GetId();
	wimaxHelper.EnableAscii(strStream.str(), container);
	
	m_ipv4AdressHelper.Assign(netDevContainer);

	// Posiciona o navio
	double x = (rota == RotaDirecao::LESTE) ? 0.0 : m_comprimentoRota;
	double velocidadeX = (rota == RotaDirecao::LESTE) ? velocidade : (-1)*velocidade;
	Vector3D vetorPosicao(x, CalculaYNovoNavio(rota), 0);
	Vector3D vetorVelocidade(velocidadeX, 0, 0);
	MobilityHelper mHelper;
	mHelper.SetMobilityModel("ns3::ConstantVelocityMobilityModel",
								"Position", Vector3DValue(vetorPosicao),
								"Velocity", Vector3DValue(vetorVelocidade));
	mHelper.Install(container);

	// Configura a aplicacao cliente udp
	UdpClientHelper clientHelper(m_ipInterfaceContainerEstacaoTerrestre.GetAddress(0), UDP_SERVER_PORT);
	clientHelper.SetAttribute("MaxPackets", UintegerValue(1500)); // TODO randomizar ?
	clientHelper.SetAttribute("Interval", TimeValue(Seconds(0.5))); // TODO randomizar ?
	clientHelper.SetAttribute("PacketSize", UintegerValue(1024)); // TODO randomizar ?
	ApplicationContainer applicationContainer = clientHelper.Install(container);
	applicationContainer.Start(Seconds(1));
	applicationContainer.Stop(Seconds(m_tempoSimulacao));

	// Instala o internet stack
	m_internetHelper.Install(container);

	if (rota == RotaDirecao::LESTE)
	{
		m_rotaLesteNodeContainer.Add(container);
	}
	else
	{
		m_rotaOesteNodeContainer.Add(container);
	}
}

double Experiment::CalculaYNovoNavio(RotaDirecao rota)
{
	double y = 0.0;
	UniformRandomVariable randomVariable;
	if (rota == RotaDirecao::LESTE)
	{
		y = m_distanciaEstacaoTerrestre + randomVariable.GetValue(1.0, m_comprimentoRota - 1.0);
		if (m_ultimoNavioRotaLesteY < 0)
		{
			m_ultimoNavioRotaLesteY = y;
			return y;
		}
		double absDistanciaDoAnterior = abs(m_ultimoNavioRotaLesteY - y);
		if (absDistanciaDoAnterior < m_distanciaMinimaEntreNavios)
		{
			if (y + (m_distanciaMinimaEntreNavios - absDistanciaDoAnterior) > m_distanciaEstacaoTerrestre + m_larguraRotas)
			{
				y -= m_distanciaMinimaEntreNavios - absDistanciaDoAnterior;
			}
			else
			{
				y += m_distanciaMinimaEntreNavios - absDistanciaDoAnterior;
			}
		}
		m_ultimoNavioRotaLesteY = y;
	}
	else
	{
		y = m_distanciaEstacaoTerrestre + m_comprimentoRota + randomVariable.GetValue(1.0, m_comprimentoRota - 1.0);
		if (m_ultimoNavioRotaOesteY < 0)
		{
			m_ultimoNavioRotaOesteY = y;
			return y;
		}
		double absDistanciaDoAnterior = abs(m_ultimoNavioRotaOesteY - y);
		if (absDistanciaDoAnterior < m_distanciaMinimaEntreNavios)
		{
			if (y + (m_distanciaMinimaEntreNavios - absDistanciaDoAnterior) > m_distanciaEstacaoTerrestre + m_larguraRotas*2)
			{
				y -= m_distanciaMinimaEntreNavios - absDistanciaDoAnterior;
			}
			else
			{
				y += m_distanciaMinimaEntreNavios - absDistanciaDoAnterior;
			}
		}
		m_ultimoNavioRotaOesteY = y;
	}
	return y;
}

Ptr<WimaxChannel> Experiment::CriaWimaxChannel()
{
	Ptr<SimpleOfdmWimaxChannel> wimaxChannel = CreateObject<SimpleOfdmWimaxChannel>();
	wimaxChannel->SetPropagationModel(SimpleOfdmWimaxChannel::FRIIS_PROPAGATION);
	return wimaxChannel;
}

int main(int argc, char** argv)
{
	Experiment experimento;
	experimento.CommandSetup(argc, argv);
	experimento.Run();
	return 0;
}