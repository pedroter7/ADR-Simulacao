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

#define EXPERIMENT_NAME "Experimento_ADR"
#define PROTOCOLO_AODV 1
#define PROTOCOLO_OLSR 2
#define PROTOCOLO_AODV_NOME "AODV"
#define PROTOCOLO_OLSR_NOME "OLSR"

using namespace ns3;

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

	void SetNomeProtocolo();

};

Experiment::Experiment() 
    :   m_protocolo(PROTOCOLO_AODV),
        m_nomeProtocolo(PROTOCOLO_AODV_NOME)
{

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

void Experiment::CommandSetup(int argc, char **argv)
{
	CommandLine cmd(__FILE__);
	cmd.AddValue("output-dir", "Diretorio para escrever os resultados da simulacao", m_outDir);
	cmd.AddValue("protocol", "1=AODV;2=OLSR", m_protocolo);
	cmd.AddValue("tempo-simulacao", "Tempo de simulacao", m_tempoSimulacao);
	cmd.AddValue("distancia-ls", "Distancia da estacao terrestre ate a rota mais proxima", m_distanciaEstacaoTerrestre);
	cmd.AddValue("comprimento-rotas", "Comprimento das rotas, i.e. comprimento da area de trafego", m_comprimentoRota);
	cmd.AddValue("largura-rotas", "Largura das rotas", m_larguraRotas);
	cmd.Parse(argc, argv);

	SetNomeProtocolo();
}

void Experiment::Run()
{
	// Configurar estacao terrestre
	// Inciar simulacao
	// Instanciar navios nas rotas
	// Escrever resultados
	// Limpar memoria	
}