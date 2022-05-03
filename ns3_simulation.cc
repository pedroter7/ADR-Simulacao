/*
    Simulacao de uma rede mesh utilizando o simulador ns3.
    Essa simulacao eh parte do trabalho da disciplina
    Avaliacao de Desempenho de Redes da UFABC
    Prof. Carlo Kleber, Q1-2022

    Essa simulacao utiliza como base pedacos de codigo das seguintes fontes:

    mesh network example: https://www.nsnam.org/doxygen/mesh_8cc_source.html
    manet routing compare example: https://www.nsnam.org/doxygen/manet-routing-compare_8cc_source.html
*/

#include<string>
#include<ns3/core-module.h>

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