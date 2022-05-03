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
#include <ns3/core-module.h>
#include <ns3/node-container.h>
#include <ns3/wimax-helper.h>
#include <ns3/net-device-container.h>

#define EXPERIMENT_NAME "Experimento_ADR"
#define PROTOCOLO_AODV 1
#define PROTOCOLO_OLSR 2
#define PROTOCOLO_AODV_NOME "AODV"
#define PROTOCOLO_OLSR_NOME "OLSR"
#define DEFAULT_LARGURA_ROTAS 20.0
#define DEFAULT_DISTANCIA_ESTACAO_TERRESTRE 10.0
#define DEFAULT_DISTANCIA_MINIMA_ENTRE_NAVIOS 5.0
#define DEFAULT_COMPRIMENTO_ROTA 60.0

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

	void ConfigurarSimulacao();
	void SetNomeProtocolo();
	void ConfigurarEstacaoTerrestre();
	void ConfigurarRota(RotaDirecao rota);
	void ConfigurarInternetStack();
	void ConfigurarLog();
	void InserirNavioNaRota(RotaDirecao rota, float velocidade);
};

Experiment::Experiment() 
    :   m_protocolo(PROTOCOLO_AODV),
        m_nomeProtocolo(PROTOCOLO_AODV_NOME),
		m_larguraRotas(DEFAULT_LARGURA_ROTAS),
		m_distanciaEstacaoTerrestre(DEFAULT_DISTANCIA_ESTACAO_TERRESTRE),
		m_comprimentoRota(DEFAULT_COMPRIMENTO_ROTA),
		m_distanciaMinimaEntreNavios(DEFAULT_DISTANCIA_MINIMA_ENTRE_NAVIOS)
{

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
	// TODO
	// Inciar simulacao
	// Instanciar navios nas rotas
	// Escrever resultados
	// Limpar memoria	
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
	ConfigurarEstacaoTerrestre();
	ConfigurarRota(RotaDirecao::LESTE);
	ConfigurarRota(RotaDirecao::OESTE);
}

void Experiment::ConfigurarLog()
{
	// TODO
}

void Experiment::ConfigurarEstacaoTerrestre()
{
	m_estacaoTeNoderrestreNodeContainer.Create(1);
	WimaxHelper wimaxHelper;
	NetDeviceContainer netDevContainer;
	netDevContainer = wimaxHelper.Install(m_estacaoTeNoderrestreNodeContainer,
											WimaxHelper::DEVICE_TYPE_BASE_STATION,
											WimaxHelper::SIMPLE_PHY_TYPE_OFDM,
											WimaxHelper::SCHED_TYPE_SIMPLE);
	wimaxHelper.EnableAscii("land-station", m_estacaoTeNoderrestreNodeContainer);
	// configurar o raio de transmissao

	// TODO
}

void Experiment::ConfigurarRota(RotaDirecao rota)
{
	// TODO
}

void Experiment::ConfigurarInternetStack()
{
	// TODO
}

void Experiment::InserirNavioNaRota(RotaDirecao rota, float velocidade)
{
	// TODO
}

int main(int argc, char** argv)
{
	Experiment experimento;
	experimento.CommandSetup(argc, argv);
	experimento.Run();
	return 0;
}