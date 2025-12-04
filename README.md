# Simulação de USS - Desconflito Estratégico 4D

Este projeto implementa uma simulação de um **UAS Service Supplier (USS)** com foco no **desconflito estratégico 4D** de trajetórias de drones. O sistema simula a interação entre Operadores e o USS, validando intenções de voo antes da decolagem.

Desenvolvido como projeto final da disciplina de **Simulação de Drones e Aplicações** (Mestrado - ITA).

## 🎯 Objetivo

Simular um ambiente UTM (Unmanned Traffic Management) onde múltiplos operadores submetem planos de voo (Intenções Operacionais). O sistema utiliza um algoritmo de busca com **resolução adaptativa** para garantir que novas rotas não conflitem com operações já aprovadas no espaço (3D) e no tempo.

## 🚀 Funcionalidades Principais

* **Interface Operador vs. USS:** Dashboard interativo desenvolvido em Streamlit.
* **Algoritmo `TacticalNavigator`:** Motor de verificação de rotas que utiliza **H3 (Uber DGGS)** com resolução adaptativa (Zoom-in em áreas de risco) para detecção eficiente de conflitos.
* **Desconflito Estratégico:** Validação de trajetórias 4D (Latitude, Longitude, Altitude, Tempo).
* **Resolução de Conflitos:** O USS sugere automaticamente alterações de altitude ou janela de tempo caso a rota solicitada esteja bloqueada.
* **Visualização Geoespacial:** Renderização das rotas aprovadas em mapa 3D interativo via PyDeck.

## 📂 Estrutura do Projeto

```text
projeto_uss_sim/
│
├── app.py                   # Interface do usuário (Streamlit Frontend)
├── config.py                # Configurações globais
├── requirements.txt         # Dependências do projeto
│
├── core/                    # Lógica de Negócio (Backend)
│   ├── __init__.py
│   ├── models.py            # Dataclasses (FlightIntent, Waypoint)
│   ├── uss_engine.py        # Motor de decisão do USS
│   ├── geo_algorithms.py    # Script de busca geográfica vetorial
│   ├── obstacle_model.py    # Carregamento do kml e da classe H3ObstacleModel
│   └── hcp.py               # Algoritmo de busca/verificação
│
├── tests/                    # Testes
│   ├── __init__.py           # Arquivo vazio
│   └── *.py                  # Scripts de testes
│
└── utils/                   # Utilitários
    ├── __init__.py
    └── geo_utils.py         # Conversões e helpers H3