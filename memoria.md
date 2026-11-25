# Memória do Projeto 6 — Robótica Móvel (PPGEE0060 / UFBA)

> **Propósito deste Documento:** Este arquivo é a memória viva, o caderno de decisões arquiteturais e o guia mestre fase a fase do **Projeto 6 (Etapa I + Etapa II)** da disciplina **PPGEE0060 — Robótica Móvel** (PPGEEC/UFBA, Prof. Dr. André Gustavo Scolari Conceição).
> Ele também registra como cada componente construído aqui se conecta diretamente à **Dissertação de Mestrado** (TinyML em STM32F411 Blackpill, geração de *dataset* por *Behavioral Cloning* e bancada *Hardware-in-the-Loop*).

**Última atualização:** 2026-11-20
**Fase atual:** `Fase 5 — Etapa II no Simulador: APF vs. Bug2 (50% da nota)`

---

## 1. Visão Geral, Pergunta Experimental e Critérios de Avaliação

### 1.1 Escopo do Projeto (Edital 2026/2)
* **Etapa I (Obrigatória):** Modelagem cinemática e localização por odometria de um robô móvel com acionamento diferencial em ambiente simulado, a partir de medidas equivalentes a encoders das rodas, comparada ao *ground truth* (usado **exclusivamente** para validação).
* **Etapa II (Tema Escolhido — P6):** Navegação reativa e desvio de obstáculos comparando duas estratégias clássicas sem mapa global prévio:
  * **Método A:** Campos Potenciais Artificiais (**APF** — *Artificial Potential Fields*).
  * **Método B:** Algoritmo **Bug2**.

### 1.2 Pergunta Experimental Verificável (Tema P6)
> *Em um robô móvel diferencial navegando exclusivamente com odometria incremental (sujeita a deriva por ruído) e LiDAR 2D local, qual o compromisso quantitativo entre suavidade/comprimento de trajetória e garantia de convergência ao comparar Campos Potenciais Artificiais (APF) e Bug2 em ambientes com obstáculos convexos esparsos, passagens estreitas, obstáculos próximos à meta (GNRON) e armadilhas côncavas em "U"?*

### 1.3 Cobertura de 100% da Avaliação (Tabela 2 e Seção 6 do Edital)

| Critério de Avaliação (Tabela 2 do Edital) | Peso | Fase do Nosso Guia | Status |
| :--- | :---: | :--- | :---: |
| **1. Implementação e validação da odometria (Etapa I)** | **20%** | Fases 1.1, 2, 3 e 4 | `[ ]` Em estudo (Fase 1.1) |
| **2. Implementação ou integração da aplicação (Etapa II — P6)** | **50%** | Fases 1.2, 1.3, 2 e 5 | `[ ]` Pendente |
| **3. Metodologia experimental e avaliação quantitativa** | **10%** | Fases 5 e 6 | `[ ]` Pendente |
| **4. Discussão técnica e interpretação dos resultados** | **10%** | Fases 6 e 7 | `[ ]` Pendente |
| **5. Apresentação e demonstração final** | **10%** | Fase 7 | `[ ]` Pendente |
| **TOTAL** | **100%** | **Fases 0 a 7** | **Meta: 100%** |

#### Entregáveis Obrigatórios (Seção 6 do Edital)
- [ ] **E1:** Código-fonte desenvolvido e arquivos de configuração (identificando bibliotecas, versões e alterações).
- [ ] **E2:** Modelo do robô, cenários de simulação (`.sdf`) e instruções de execução reprodutível (`README.md`).
- [ ] **E3:** Resultados da Etapa I (3 trajetórias de teste + comparação da odometria com o *ground truth*).
- [ ] **E4:** Artigo científico no formato **IEEE Conference Template (`IEEEconf`)**, redigido com autoria própria (Título, Resumo, Introdução, Fundamentação Teórica e Trabalhos Relacionados, Metodologia, Desenvolvimento e Implementação, Resultados Experimentais, Discussão, Conclusões e Referências).
- [ ] **E5:** Gráficos e tabelas de avaliação (Tabela 1 do edital: Taxa de Sucesso, Colisões, Comprimento, Tempo, $d_{min}$, Bloqueios/Mínimos Locais e métricas de Odometria/RMSE).
- [ ] **E6:** Apresentação oral com demonstração funcional ao vivo no Gazebo.

---

## 2. Registro de Decisões Técnicas (ADR — *Architecture Decision Records*)

| ID | Decisão | Justificativa Técnica e Regra Associada | Origem |
| :---: | :--- | :--- | :--- |
| **D1** | **Plataforma de Simulação e Middleware:** **Gazebo Harmonic** (`/usr/bin/gz`) + **ROS 2 Jazzy Jalisco** (`/opt/ros/jazzy`) + Python (`rclpy`) + `ros_gz_bridge` | Evita retrabalho metodológico na dissertação: mesmo simulador, formato de mensagens (`/scan`, `/cmd_vel`, `/joint_states`) e infraestrutura que serão usados na geração do *dataset* e no *Hardware-in-the-Loop* (HIL). | [Adequação de Projeto ao Mestrado.pdf](file:///mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Adequação%20de%20Projeto%20ao%20Mestrado.pdf) |
| **D2** | **Modelo do Robô e Origem Geométrica (`base_link`):** Robô diferencial com raio $R = 0{,}05\text{ m}$ e bitola $L = 0{,}35\text{ m}$, com `base_link` **exatamente no ponto médio do eixo entre as rodas motrizes** $(0, 0)$ | A restrição não-holonômica ($\dot{y}_R = 0$) é válida apenas no ponto médio do eixo motriz. Deslocar o `base_link` do eixo (como ocorria no protótipo `robo_simples.sdf`) gera erro artificial de braço de alavanca em curvas ao comparar com o *ground truth*. | [Adequação de Projeto ao Mestrado.pdf](file:///mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Adequação%20de%20Projeto%20ao%20Mestrado.pdf) |
| **D3** | **Isolamento Estrito do *Ground Truth* (Regra de Ouro):** O tópico `/ground_truth/odom` será assinado **exclusivamente** pelo nó avaliador (`metrics_logger_node`) | Exigência explícita das Seções 2.1 e 5.2 do edital: nenhum nó de estimação de pose (`odom_estimator`) ou navegação (`apf`, `bug2`) pode ler a pose verdadeira do simulador. | [Projeto_Robotica_Movel_2026-2.pdf](file:///mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Projeto_Robotica_Movel_2026-2.pdf) |
| **D4** | **Integração Numérica da Odometria:** Método do Ponto Médio (**Runge-Kutta de 2ª Ordem — RK2**) a $50\text{ Hz}$ ($\Delta t = 0{,}02\text{ s}$) com `use_sim_time: true` e normalização angular $\text{wrapToPi}(\theta) \in (-\pi, \pi]$ | Possui erro de truncamento local $\mathcal{O}(\Delta t^3)$, reduzindo drasticamente o erro em trajetórias curvilíneas em comparação ao método de Euler explícito $\mathcal{O}(\Delta t^2)$. Usa relógio de simulação `/clock` para sincronia. | [Guia Teórico Completo - Fase 1.pdf](file:///mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Guia%20Teórico%20Completo%20-%20Fase%201_%20Fundamentos%20de%20Robótica%20Móvel,%20Odometria,%20APF%20e%20Bug2.pdf) |
| **D5** | **Sensores e QoS:** Encoders em `/joint_states` ($100\text{ Hz}$) e LiDAR 2D planar $360^\circ$ ($360$ feixes, $[0{,}12\text{ m}, 8{,}0\text{ m}]$, $20\text{ Hz}$) em `/scan` com `qos_profile_sensor_data` | Garante compatibilidade com o plugin `gpu_lidar` via `ros_gz_bridge` sem perda silenciosa de pacotes por incompatibilidade de QoS (*Best Effort* vs. *Reliable*). | [Adequação de Projeto ao Mestrado.pdf](file:///mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Adequação%20de%20Projeto%20ao%20Mestrado.pdf) |
| **D6** | **Protocolo Experimental e Repetibilidade:** 3 trajetórias na Etapa I (Reta, Arco, Composta em "S"/Quadrado); 2 Cenários principais na Etapa II (A: Esparsos/Corredor + GNRON; B: Armadilha em "U") com **5 repetições** por condição usando sementes fixas ($s \in \{42,43,44,45,46\}$) e ruído gaussiano configurável | Em simuladores físicos determinísticos, 5 rodadas idênticas teriam $\sigma = 0$. A injeção controlada de ruído com sementes registradas atende à Seção 5.1 do edital e gera estatísticas reais ($\mu \pm \sigma$) na Tabela 1. | [Projeto_Robotica_Movel_2026-2.pdf](file:///mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Projeto_Robotica_Movel_2026-2.pdf) e [Adequação](file:///mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Adequação%20de%20Projeto%20ao%20Mestrado.pdf) |
| **D7** | **Metodologia de Desenvolvimento e Autoria:** Aprendizado incremental passo a passo do zero (Teoria no papel $\to$ Python 2D puro $\to$ Gazebo/ROS 2 $\to$ Experimentos $\to$ Artigo escrito pelo aluno) | Garante domínio teórico-prático completo para a defesa oral na disciplina, autoria real do artigo IEEE e base sólida para a qualificação e dissertação de mestrado. | Alinhado na Fase 0 |

---

## 3. Biblioteca de Referência e Roteiro de Leitura Direcionada

### 3.1 Documentos da Disciplina e Guias
* **Edital Oficial:** [Projeto_Robotica_Movel_2026-2.pdf](file:///mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Projeto_Robotica_Movel_2026-2.pdf)
* **Adequação ao Mestrado e Arquitetura:** [Adequação de Projeto ao Mestrado.pdf](file:///mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Adequação%20de%20Projeto%20ao%20Mestrado.pdf)
* **Apostila Base (Fase 1):** [Guia Teórico Completo - Fase 1.pdf](file:///mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Guia%20Teórico%20Completo%20-%20Fase%201_%20Fundamentos%20de%20Robótica%20Móvel,%20Odometria,%20APF%20e%20Bug2.pdf)

### 3.2 Livros e Artigos Científicos (O que ler em cada um)

| Módulo | Fonte / Link Direto | Seções / Páginas Alvo | Conceito-Chave a Extrair |
| :---: | :--- | :--- | :--- |
| **1.1** | **Siegwart & Nourbakhsh (2004)** — [Introduction to Autonomous Mobile Robots.pdf](file:///mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/40658890-Introduction-to-Autonomous-Mobile-Robots-_260828_133851.pdf) | • Cap. 3: Seções **3.2.1 e 3.2.2** (p. 48–53 do livro / p. 63–68 do PDF)<br>• Cap. 3: Seções **3.2.3 e 3.2.4** (p. 53–63)<br>• Cap. 5: Seção **5.2.4** (p. 186–191) | Referencial inercial vs. local, matriz de rotação $R(\theta)$, modelo cinemático diferencial, restrição de rolamento puro sem escorregamento lateral e modelo de erro odométrico. |
| **1.1** | **Peter Corke (2023)** — [Robotics, Vision and Control in Python.pdf](file:///mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Artigos/%5BSpringer%20Tracts%20in%20Advanced%20Robotics%20%E2%84%96146%5D%20Peter%20Corke%20-%20Robotics,%20Vision%20and%20Control.%20Fundamental%20Algorithms%20in%20Python%20(2023,%20Springer)%20%5B10.1007_978-3-031-06469-2%5D%20-%20libgen.li.pdf) | • Cap. 4: *Mobile Robot Vehicles* (Seções **4.1.2** *Differentially-Steered Vehicle* e **4.3.1** *Nonholonomic Systems*) | Discretização em passos $\Delta t$, integração numérica em Python e representação prática da cinemática diferencial. |
| **1.2** | **Khatib (1986)** — [Khatib_1986_IJRR.pdf](file:///mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Artigos/Khatib_1986_IJRR.pdf) | • Seções iniciais de formulação do *Artificial Potential Field* | Definição da função potencial atrativa e repulsiva (FIRAS) e obtenção da força virtual pelo gradiente negativo ($\mathbf{F} = -\nabla U$). |
| **1.2** | **Koren & Borenstein (1991)** — [borenstein_potential_field_limitations.pdf](file:///mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Artigos/borenstein_potential_field_limitations.pdf) | • Artigo completo (pp. 1398–1404, 7 páginas) | As **4 limitações inerentes** do APF: (1) Mínimos locais em armadilhas côncavas; (2) Impossibilidade de passar entre obstáculos próximos; (3) Oscilações na presença de obstáculos; (4) Oscilações em corredores estreitos (e base do fenômeno GNRON). |
| **1.3** | **Lumelsky & Stepanov (1987)** — [PathPlanningStrategies.pdf](file:///mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Artigos/PathPlanningStrategies.pdf) | • Seções de formulação e algoritmos **Bug1** e **Bug2** | Definição da linha-$m$, pontos de impacto ($H_i$) e saída ($L_i$), condição de progresso estrito $\|L_i - T\| < \|H_i - T\|$ e prova de convergência em tempo finito. |

> **Dica de Notação ao Ler o Siegwart (2004):** No Cap. 3.2.2 do Siegwart, ele usa $r$ para raio da roda, $l$ para **metade** da distância entre rodas (logo, nossa bitola total é $L = 2l$), $\dot{\varphi}_1$ para roda direita ($\omega_d$) e $\dot{\varphi}_2$ para roda esquerda ($\omega_e$). Tenha essa conversão em mente ao comparar com as nossas equações!

---

## 4. Ponte Direta com a Dissertação de Mestrado (PPGEEC)

1. **Etapa I (Cinemática e Odometria RK2)** $\longrightarrow$ Será portada para linguagem C (`float` / ponto fixo) no firmware da **STM32F411 Blackpill** para ler os encoders físicos e atualizar $(x_t, y_t, \theta_t)$ e o vetor polar da meta $(d_t, \alpha_t)$ usando pouquíssimos bytes de SRAM.
2. **Etapa II (Controladores APF e Bug2 no Gazebo)** $\longrightarrow$ Servirão como **agentes especialistas** no Gazebo para gerar automaticamente dezenas de milhares de amostras (*Behavioral Cloning*) com o sensor matricial ToF $8\times 8$ (VL53L5CX), além de fornecerem a justificativa experimental da dissertação (por que métodos reativos clássicos falham em mínimos locais ou geram caminhos subótimos sob deriva odométrica).
3. **Nó Avaliador e Tabela 1 (Métricas)** $\longrightarrow$ Serão reutilizados nos mesmos cenários no Gazebo para a bancada **Hardware-in-the-Loop (HIL)** conectada via serial USB à Blackpill executando a rede TinyML quantizada (QAT/LSQ).

---

## 5. Guia de Fases e Checklist de Progresso

### Fase 0 — Memória e Alinhamento
- [x] Plano de mentoria aprovado.
- [x] `memoria.md` criado com decisões, critérios do edital e links bibliográficos.

### Fase 1 — Domínio da Teoria (No Papel, Antes do Código)
- [x] **Módulo 1.1:** Referenciais, restrição não-holonômica, cinemática diferencial ($v, \omega$), Euler vs. RK2, deriva angular e métricas ($e_p, e_\theta, \text{RMSE}$).
- [x] **Módulo 1.2:** APF de Khatib ($\mathbf{F}_{att}, \mathbf{F}_{rep}$ por feixe LiDAR, projeção para $(v, \omega)$) e as 4 limitações de Koren & Borenstein (mínimos locais, GNRON, oscilações).
- [x] **Módulo 1.3:** Algoritmo Bug2 de Lumelsky & Stepanov (Linha-$m$, $d_m(\mathbf{q})$, FSM `GOAL_SEEKING` $\leftrightarrow$ `WALL_FOLLOWING`, lei polar tangencial, $H_i, L_i$ e condição de saída).

### Fase 2 — Laboratório Matemático 2D (Python Puro, Sem ROS/Gazebo)
- [x] `passo1_cinematica_rk2.py`: Implementar `wrap_to_pi` e `passo_odometria_rk2`; testar reta, arco e erro de Euler vs. RK2.
- [x] `passo2_apf_2d.py`: Simular partícula/robô 2D com `matplotlib` desviando de obstáculos e reproduzir mínimo local em "U" e GNRON.
- [x] `passo3_bug2_2d.py`: Simular máquina de estados Bug2 em 2D escapando da armadilha em "U".

### Fase 3 — Infraestrutura Gazebo Harmonic + ROS 2 Jazzy
- [x] Entender o SDF do robô diferencial (`base_link` no centro do eixo, $R=0{,}05\text{ m}$, $L=0{,}35\text{ m}$) e os 4 plugins (`diff-drive`, `joint-state-publisher`, `gpu_lidar`, `odometry-publisher`).
- [x] Configurar o pacote ROS 2 e o `bridge_params.yaml` (`ros_gz_bridge`).
- [x] Testar movimentação com `teleop_twist_keyboard` e inspecionar `/joint_states`, `/scan` e `/ground_truth/odom` (confirmando isolamento).

### Fase 4 — Etapa I no Simulador: Odometria (20% da nota)
- [ ] Implementar `odom_estimator_node.py` (com `use_sim_time: true` e ruído configurável).
- [ ] Implementar `etapa1_test_driver.py` (Reta, Arco circular, Composto em "S"/Quadrado) e `metrics_logger_node.py`.
- [ ] Gerar e interpretar os gráficos comparativos (Estimado vs. *Ground Truth*) e $\text{RMSE}_p, \text{RMSE}_\theta$.

### Fase 5 — Etapa II no Simulador: APF vs. Bug2 (50% da nota)
- [ ] Construir `cenario_a.sdf` (obstáculos esparsos, corredor + variação GNRON) e `cenario_b.sdf` (armadilha em "U").
- [ ] Implementar `apf_navigator_node.py` e validar visualmente no Gazebo (incluindo detecção de mínimo local e GNRON).
- [ ] Implementar `bug2_navigator_node.py` e validar visualmente o escape no Cenário B (marcando $H_i$ e $L_i$).

### Fase 6 — Validação Experimental Científica e Métricas (20% da nota: Metodologia + Discussão)
- [ ] Executar estudo paramétrico do APF ($\rho_0, k_{att}, k_{rep}$) e experimento GNRON.
- [ ] Executar bateria de **5 repetições** por condição ($s \in \{42,43,44,45,46\}$) nos Cenários A e B.
- [ ] Consolidar Tabela 1 de métricas ($\mu \pm \sigma$) e gerar todas as figuras em alta resolução para o artigo.

### Fase 7 — Redação Autoral do Artigo IEEE e Defesa Oral (10% da nota)
- [ ] Redigir em LaTeX (`IEEEtran` / `IEEEconf`) seção por seção com autoria própria e revisão conjunta.
- [ ] Preparar `README.md` reprodutível, roteiro de apresentação oral e ensaiar demonstração ao vivo no Gazebo.

---

## 6. Diário de Bordo e Aprendizados

* **2026-11-17 (Fase 0):** Plano de mentoria aprovado e `memoria.md` criado. Ferramentas verificadas no ambiente local (`/opt/ros/jazzy`, `/usr/bin/gz`, `/usr/bin/pdflatex`, `/usr/bin/python3`). Iniciando **Fase 1 — Módulo 1.1**.
* **2026-11-18 (Fase 1 — Módulo 1.1 $\to$ 1.2):** Revisados referenciais, restrição não-holonômica, cinemática diferencial ($v, \omega$), sinal da curvatura, integração Euler vs. RK2 e justificativa geométrica de `base_link`. Avançando para **Fase 1 — Módulo 1.2 (APF e limitações de Koren & Borenstein)**.
* **2026-11-18 (Fase 1 — Módulo 1.2 $\to$ 1.3):** Revisado APF (Khatib 1986), cálculo vetorial de $\mathbf{F}_{att}$ e $\mathbf{F}_{rep}$ via LiDAR, projeção cinemática com frenagem $\max(0, \cos e_{\theta,d})$ e as 4 limitações de Koren & Borenstein (1991) + GNRON. Avançando para **Fase 1 — Módulo 1.3 (Algoritmo Bug2)**.
* **2026-11-18 (Fase 1 Concluída $\to$ Fase 2):** Concluído o Módulo 1.3 (Bug2, Linha-$m$, FSM e lei polar tangencial). Iniciando **Fase 2 — Laboratório Matemático 2D em Python Puro** com `passo1_cinematica_rk2.py`.
* **2026-11-19 (Fase 2 — Passo 2.1 $\to$ 2.2):** Validado `passo1_cinematica_rk2.py` (Euler vs. RK2 e deriva em "S"). Iniciando **Passo 2.2 (`passo2_apf_2d.py`)** para simular APF com LiDAR 2D, mínimo local em "U" e GNRON.
* **2026-11-19 (Fase 2 — Passo 2.2 $\to$ 2.3):** Validado `passo2_apf_2d.py` (sucesso no Cenário A, mínimo local em "U" com $\|\mathbf{F}_{total}\|=0{,}000$ no Cenário B e GNRON a $0{,}354\text{ m}$ da meta). Iniciando **Passo 2.3 (`passo3_bug2_2d.py`)**.
* **2026-11-19 (Fase 2 Concluída $\to$ Fase 3):** Validado `passo3_bug2_2d.py` (100% sucesso nos Cenários A, B e GNRON). Iniciando **Fase 3 — Infraestrutura Gazebo Harmonic + ROS 2 Jazzy** (robô `.sdf`, 4 plugins e ponte `ros_gz_bridge`).
* **2026-11-20 (Fase 3 Concluída $\to$ Fase 4):** Robô diferencial modular (`model://robo_diferencial`), `ros_gz_bridge` e sensores (`/joint_states`, `/scan` com `best_effort`, `/ground_truth/odom`) validados no Gazebo Harmonic 8.11.0 com teleoperação. Iniciando **Fase 4 — Etapa I (Odometria RK2 + 3 Trajetórias + RMSE)**.
