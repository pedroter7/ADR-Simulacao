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
#include <ns3/mobility-model.h>
#include <ns3/flow-monitor.h>
#include <ns3/flow-monitor-helper.h>
#include <ns3/nstime.h>
#include <ns3/internet-stack-helper.h>
#include <utility>
#include <cmath>
#include <numbers>
#include <mutex>
#include <fstream>
#include <vector>
#include <algorithm>

#ifdef _WIN32 || _WIN64 || __CYGWIN__
	#define PATH_SEPARATOR "\"
#else
	#define PATH_SEPARATOR "/"
#endif

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
#define MAX_PACOTES_HORA 450000000
#define PACKET_SIZE 1024
// Constantes das funcoes densidade de probabilidade
#define FDP_A_ROTA_OESTE 0.292064
#define FDP_B_ROTA_OESTE 0.998286
#define FDP_C_ROTA_OESTE 4.763146
#define FDP_MU_ROTA_OESTE 10.383780
#define FDP_SIGMA_ROTA_OESTE 5.488477
#define FDP_A_ROTA_LESTE 0.284495
#define FDP_B_ROTA_LESTE 0.998324
#define FDP_C_ROTA_LESTE 4.354804
#define FDP_MU_ROTA_LESTE 10.800865
#define FDP_SIGMA_ROTA_LESTE 5.189037
#define MATH_PI 3.141592
#define VELOCIDADE_MAX_NAVIO 27.6874 // km por segundo de simulacao
// Constantes para converter 1 segundo simulacao = 1 hora
#define FRACAO_SIMULACAO_MINUTO 0.016
#define FRACAO_SIMULACAO_SEGUNDO 0.00028

using namespace ns3;

enum RotaDirecao
{
	LESTE,
	OESTE
};

typedef struct MetricasColetadas
{
	double segundosSimulacao;
	uint32_t naviosNasRotas;
	double throughput;
	double endToEndDelay;
} MetricasColetadas;

typedef struct ComparaCdfs
{
	bool operator() (const std::pair<double, double> & a, const std::pair<double, double> & b)
	{
		return a.second < b.second;
	}
} ComparaCdfs;

NS_LOG_COMPONENT_DEFINE(EXPERIMENT_NAME);

class Experiment
{
public:
	Experiment();
	void CommandSetup(int argc, char **argv);
	void Run();
private:
	std::mutex m_mutex;
	uint32_t m_protocolo;
	std::string m_nomeProtocolo;
	std::string m_outDir;
	std::string m_caminhoArquivoResultados;
	std::ofstream m_streamArquivoResultados;
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
	EmpiricalRandomVariable m_variavelTempoEntreNaviosLeste;
	EmpiricalRandomVariable m_variavelVelocidadeNaviosLeste;
	EmpiricalRandomVariable m_variavelTempoEntreNaviosOeste;
	EmpiricalRandomVariable m_variavelVelocidadeNaviosOeste;
	// Controle para nao inserir um navio em cima de outro
	double m_ultimoNavioRotaLesteY;
	double m_ultimoNavioRotaOesteY;
	// Metricas
	uint32_t m_totalNaviosNaRota;
	Ptr<FlowMonitor> m_flowMonitor;

	void ConfigurarSimulacao();
	void SetNomeProtocolo();
	void ConfigurarEstacaoTerrestre();
	void ConfigurarRoteamento();
	void ConfigurarLog();
	void InserirNavioNaRota(RotaDirecao rota, double velocidade);
	Ptr<WimaxChannel> CriaWimaxChannel();
	double CalculaYNovoNavio(RotaDirecao rota);
	void ConfigurarDistribuicoes();
	void ConfigurarVariavelTempoNavios();
	void ConfigurarVariavelVelocidadeNavios();
	void ConfigurarMetricas();
	double FdpTempoNavios(double x, RotaDirecao rota);
	double FdpVelocidadeNavios(double x, RotaDirecao rota);
	void IniciarRotinas();
	void RotinaInstanciarNavioRotaLeste();
	void RotinaInstanciarNavioRotaOeste();
	void RotinaColetaDeMetricas();
	// Arquivo de resultados
	void SetCaminhoArquivoResultados();
	void EscreverHeaders();
	void EscreverMetricasColetadas(MetricasColetadas metricas);
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
		m_ultimoNavioRotaOesteY(-1.0),
		m_variavelTempoEntreNaviosLeste(),
		m_variavelVelocidadeNaviosLeste(),
		m_variavelTempoEntreNaviosOeste(),
		m_variavelVelocidadeNaviosOeste(),
		m_totalNaviosNaRota(0),
		m_streamArquivoResultados()
{
	m_ipv4AdressHelper.SetBase("10.1.0.0", "255.255.0.0");
}

void Experiment::SetCaminhoArquivoResultados()
{
	std::stringstream st;
	st << m_outDir << PATH_SEPARATOR
		<< "ResultadoSimulacao_"
		<< m_nomeProtocolo << "_MeshNetwork.csv";

	m_caminhoArquivoResultados = st.str();
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

void Experiment::EscreverHeaders()
{
	if (m_streamArquivoResultados.is_open()) m_streamArquivoResultados.close();

	m_streamArquivoResultados.open(m_caminhoArquivoResultados);
	
	m_streamArquivoResultados << "SimulationSecond,"
								<< "NodesNumber,"
								<< "Throughput(bytes/s),"
								<< "EndToEndDelay(s),"
								<< "Protocol"
								<< std::endl;

	m_streamArquivoResultados.close();
}

void Experiment::EscreverMetricasColetadas(MetricasColetadas metricas)
{
	if (!m_streamArquivoResultados.is_open()) 
		m_streamArquivoResultados.open(m_caminhoArquivoResultados, std::ios::app);

	m_streamArquivoResultados << metricas.segundosSimulacao
								<< ","
								<< metricas.naviosNasRotas
								<< ","
								<< metricas.throughput
								<< ","
								<< metricas.endToEndDelay
								<< ","
								<< m_nomeProtocolo
								<< std::endl;
}

void Experiment::Run()
{
	ConfigurarSimulacao();
	IniciarRotinas();
	NS_LOG_INFO("Iniciando simulacao");
	Simulator::Stop(Seconds(m_tempoSimulacao));
	Simulator::Run();

	// Clear
	m_streamArquivoResultados.close();
	Simulator::Destroy();
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
	ConfigurarRoteamento();
	ConfigurarEstacaoTerrestre();
	ConfigurarDistribuicoes();
	SetCaminhoArquivoResultados();
	EscreverHeaders();
}

void Experiment::ConfigurarLog()
{
	LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
	LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
}

void Experiment::ConfigurarEstacaoTerrestre()
{
	NS_LOG_INFO("Configurando estacao terrestre");
	m_estacaoTeNoderrestreNodeContainer.Create(1);

	WimaxHelper wimaxHelper;
	NetDeviceContainer netDevContainer;
	netDevContainer = wimaxHelper.Install(m_estacaoTeNoderrestreNodeContainer,
											WimaxHelper::DEVICE_TYPE_BASE_STATION,
											WimaxHelper::SIMPLE_PHY_TYPE_OFDM,
											CriaWimaxChannel(),
											WimaxHelper::SCHED_TYPE_SIMPLE);
	wimaxHelper.EnableAscii("land-station", m_estacaoTeNoderrestreNodeContainer);

	// Instala o internet stack
	m_internetHelper.Install(m_estacaoTeNoderrestreNodeContainer);

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

	NS_LOG_INFO("Estacao terrestre configurada");
}

void Experiment::ConfigurarRoteamento()
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
	std::stringstream s;
	s << "Inserindo navio na rota ";
	s << (rota == RotaDirecao::LESTE) ? "Leste" : "Oeste";
	NS_LOG_INFO(s.str());
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

	// Instala o internet stack
	m_internetHelper.Install(container);

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
	clientHelper.SetAttribute("MaxPackets", UintegerValue(MAX_PACOTES_HORA));
	// Randomiza intervalo entre o envio de pacotes
	UniformRandomVariable uRandVar;
	double intervaloMinimo = FRACAO_SIMULACAO_SEGUNDO/10;
	double intervaloMaximo = FRACAO_SIMULACAO_SEGUNDO*2;
	clientHelper.SetAttribute("Interval", TimeValue(Seconds(uRandVar.GetValue(intervaloMinimo, intervaloMaximo)))); 
	clientHelper.SetAttribute("PacketSize", UintegerValue(PACKET_SIZE));
	ApplicationContainer applicationContainer = clientHelper.Install(container);
	applicationContainer.Start(Seconds(1));
	applicationContainer.Stop(Seconds(m_tempoSimulacao));

	if (rota == RotaDirecao::LESTE)
	{
		m_rotaLesteNodeContainer.Add(container);
	}
	else
	{
		m_rotaOesteNodeContainer.Add(container);
	}

	NS_LOG_INFO("Navio inserido");

	// Atualiza o numero de navios na rota
	m_mutex.lock();
	m_totalNaviosNaRota++;
	m_mutex.unlock();
	
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
	std::stringstream logM;
	logM << "Y do novo navio na rota " << (rota == RotaDirecao::LESTE) ? "Leste" : "Oeste";
	logM << ": " << y;
	NS_LOG_INFO(logM.str());
	return y;
}

Ptr<WimaxChannel> Experiment::CriaWimaxChannel()
{
	Ptr<SimpleOfdmWimaxChannel> wimaxChannel = CreateObject<SimpleOfdmWimaxChannel>();
	wimaxChannel->SetPropagationModel(SimpleOfdmWimaxChannel::FRIIS_PROPAGATION);
	return wimaxChannel;
}

void Experiment::ConfigurarDistribuicoes()
{
	ConfigurarVariavelTempoNavios();
	ConfigurarVariavelVelocidadeNavios();
}

void Experiment::ConfigurarVariavelTempoNavios()
{
	// TODO metodo CDF deve receber um valor e a probabilidade de o valor ser menor que ele
	NS_LOG_INFO("Configurando variaveis aleatorias do intervalo entre navios");
	std::vector<std::pair<double, double>> cdfsLeste, cdfsOeste;
	for (double i = 1.0; i <= m_tempoSimulacao/5.0; i += 0.1)
	{
		cdfsLeste.push_back(std::pair<double, double>(i, FdpTempoNavios(i, RotaDirecao::LESTE)));
		cdfsOeste.push_back(std::pair<double, double>(i, FdpTempoNavios(i, RotaDirecao::OESTE)));
	}
	std::sort(cdfsLeste.begin(), cdfsLeste.end(), ComparaCdfs());
	std::sort(cdfsOeste.begin(), cdfsOeste.end(), ComparaCdfs());

	for (auto p : cdfsLeste)
	{
		m_variavelTempoEntreNaviosLeste.CDF(p.first, p.second);
	}
	for (auto p : cdfsOeste)
	{
		m_variavelTempoEntreNaviosLeste.CDF(p.first, p.second);
	}
	NS_LOG_INFO("Variaveis configuradas");
}

void Experiment::ConfigurarVariavelVelocidadeNavios()
{
	// TODO metodo CDF deve receber um valor e a probabilidade de o valor ser menor que ele
	NS_LOG_INFO("Configurando variaveis aleatorias da velocidade dos navios");
	std::vector<std::pair<double, double>> cdfsLeste, cdfsOeste;
	for (double i = 1.0; i <= VELOCIDADE_MAX_NAVIO; i += 1.0)
	{
		cdfsLeste.push_back(std::pair<double, double>(i, FdpVelocidadeNavios(i, RotaDirecao::LESTE)));
		cdfsOeste.push_back(std::pair<double, double>(i, FdpVelocidadeNavios(i, RotaDirecao::OESTE)));
	}
	std::sort(cdfsLeste.begin(), cdfsLeste.end(), ComparaCdfs());
	std::sort(cdfsOeste.begin(), cdfsOeste.end(), ComparaCdfs());

	for (auto p : cdfsLeste)
	{
		m_variavelVelocidadeNaviosLeste.CDF(p.first, p.second);
	}
	for (auto p : cdfsOeste)
	{
		m_variavelVelocidadeNaviosOeste.CDF(p.first, p.second);
	}
	NS_LOG_INFO("Variaveis configuradas");
}

double Experiment::FdpTempoNavios(double x, RotaDirecao rota)
{
	double a = (rota == RotaDirecao::LESTE) ? FDP_A_ROTA_LESTE : FDP_A_ROTA_OESTE;
	double b = (rota == RotaDirecao::LESTE) ? FDP_B_ROTA_LESTE : FDP_B_ROTA_OESTE;
	return a * pow(b, x);
}

double Experiment::FdpVelocidadeNavios(double x, RotaDirecao rota)
{
	if (x <= 0) return 0.0;
	double c = (rota == RotaDirecao::LESTE) ? FDP_C_ROTA_LESTE : FDP_C_ROTA_OESTE;
	double mu = (rota == RotaDirecao::LESTE) ? FDP_MU_ROTA_LESTE : FDP_MU_ROTA_OESTE;
	double sigma = (rota == RotaDirecao::LESTE) ? FDP_SIGMA_ROTA_LESTE : FDP_SIGMA_ROTA_OESTE;
	return ( c / sqrt(2 * MATH_PI * pow(sigma, 2)) ) * exp( (-1) * pow((x-mu), 2) / (2 * pow(sigma, 2)) );
}

void Experiment::RotinaInstanciarNavioRotaLeste()
{
	double v = m_variavelVelocidadeNaviosLeste.GetValue();
	InserirNavioNaRota(RotaDirecao::LESTE, v);
	double intervaloProximoNavio = m_variavelTempoEntreNaviosLeste.GetValue();
	Simulator::Schedule(Seconds(intervaloProximoNavio*FRACAO_SIMULACAO_MINUTO), &Experiment::RotinaInstanciarNavioRotaLeste, this);
}

void Experiment::RotinaInstanciarNavioRotaOeste()
{
	double v = m_variavelVelocidadeNaviosOeste.GetValue();
	InserirNavioNaRota(RotaDirecao::OESTE, v);
	double intervaloProximoNavio = m_variavelTempoEntreNaviosOeste.GetValue();
	Simulator::Schedule(Seconds(intervaloProximoNavio*FRACAO_SIMULACAO_MINUTO), &Experiment::RotinaInstanciarNavioRotaOeste, this);
}

void Experiment::RotinaColetaDeMetricas()
{
	NS_LOG_INFO("Coletando metricas");
	MetricasColetadas metricas;
	m_mutex.lock();
	metricas.naviosNasRotas = m_totalNaviosNaRota;
	m_mutex.unlock();
	metricas.segundosSimulacao = Simulator::Now().GetSeconds();
	
	// Calcula throughput e end-to-end delay
	auto flowStats = m_flowMonitor->GetFlowStats();
	int128_t sDelay = 0;
	uint64_t nDelay = 0;
	uint128_t sBytes = 0;
	uint64_t nBytes = 0;
	for (auto i = flowStats.begin(); i != flowStats.end(); ++i)
	{
		auto stats = (*i).second;
		sDelay += stats.delaySum.ToInteger(Time::Unit::S);
		sBytes += stats.txBytes;
		nBytes++;
		nDelay++;
	}
	if (nDelay > 0) metricas.endToEndDelay = ((double) sDelay/nDelay)*FRACAO_SIMULACAO_SEGUNDO;
	else metricas.endToEndDelay = 0;

	if (nBytes > 0) metricas.throughput = ((double) sBytes/nBytes)*FRACAO_SIMULACAO_SEGUNDO;
	else metricas.throughput = 0;

	EscreverMetricasColetadas(metricas);

	Simulator::Schedule(Seconds(1.0), &Experiment::RotinaColetaDeMetricas, this);

	NS_LOG_INFO("Metricas coletadas: navios("
					<<metricas.naviosNasRotas
					<<"); delay("
					<<metricas.endToEndDelay
					<<"); simulationSeconds("
					<<metricas.segundosSimulacao
					<<");");
}

void Experiment::IniciarRotinas()
{
	NS_LOG_INFO("Iniciando rotinas");
	RotinaInstanciarNavioRotaLeste();
	RotinaInstanciarNavioRotaOeste();
	RotinaColetaDeMetricas();
}

void Experiment::ConfigurarMetricas()
{
	// FlowMonitor
	NS_LOG_INFO("Configurando FlowMonitor");
	FlowMonitorHelper monitorHelper;
	m_flowMonitor = monitorHelper.InstallAll();
}

int main(int argc, char** argv)
{
	Experiment experimento;
	experimento.CommandSetup(argc, argv);
	experimento.Run();
	return 0;
}