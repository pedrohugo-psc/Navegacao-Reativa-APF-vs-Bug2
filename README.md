# Projeto 6 — Navegação Reativa (APF vs. Bug2) e Odometria RK2

**Disciplina:** PPGEE0060 — Robótica Móvel (PPGEEC / UFBA)  
**Professor:** Prof. Dr. André Gustavo Scolari Conceição  
**Autor:** Pedro Hugo  
**Tema:** Etapa I (Modelagem Cinemática e Odometria) + Etapa II — Tema P6 (Navegação Reativa e Desvio de Obstáculos: Campos Potenciais Artificiais vs. Bug2)

---

## 1. Visão Geral e Arquitetura

Este repositório contém a implementação completa, reprodutível e modular do **Projeto 6** em **ROS 2 Jazzy Jalisco** + **Gazebo Harmonic**, dividida em:

* **Etapa I (Localização Proprioceptiva):** Estimador de pose plana $(x, y, \theta)$ por integração cinemática diferencial via **Método do Ponto Médio (Runge-Kutta de 2ª Ordem — RK2)** a partir dos encoders das rodas (`/joint_states`), validado contra o *Ground Truth* em 3 trajetórias (linha reta, arco circular e percurso composto em "S").
* **Etapa II — Tema P6 (Navegação Reativa sem Mapa):** Implementação e comparação quantitativa entre **Campos Potenciais Artificiais (APF — Khatib, 1986)** e **Algoritmo Bug2 (Lumelsky & Stepanov, 1987)** usando exclusivamente `/odom_est` e LiDAR 2D (`/scan`), em 3 ambientes: Cenário A (obstáculos esparsos e corredor), Cenário B (armadilha côncava em "U") e Subcenário GNRON (obstáculo a $0{,}45\text{ m}$ atrás da meta).

### Garantias Metodológicas (Conformidade com o Edital)
1. **Isolamento Estrito do *Ground Truth* (Seções 2.1 e 5.2):** O tópico `/ground_truth/odom` é assinado **exclusivamente** pelos nós avaliadores (`metrics_logger_node` e `nav_metrics_logger_node`). Nenhum nó de odometria ou navegação lê a pose verdadeira do simulador.
2. **Geometria Cinemática Calibrada:** O robô diferencial (`model://robo_diferencial`) possui raio das rodas $R = 0{,}05\text{ m}$, bitola $L = 0{,}35\text{ m}$, referencial `base_link` exatamente no ponto médio do eixo motriz $(0, 0)$ (onde $\dot{y}_R = 0$) e colisão esférica pontual nas rodas garantindo $L_{\text{eff}} = 0{,}35\text{ m}$.
3. **Sincronização e QoS:** Todos os nós operam com `use_sim_time: true` sincronizado ao `/clock` do Gazebo, e todos os assinantes de `/scan` utilizam `qos_profile_sensor_data`.
4. **Repetibilidade Estatística (Seção 5.1):** 5 execuções por condição experimental utilizando ruído branco gaussiano calibrado nos sensores ($\sigma_\omega = 0{,}05\text{ rad/s}$ nos encoders e $\sigma_\rho = 0{,}01\text{ m}$ no LiDAR) com sementes aleatórias fixas $s \in \{42, 43, 44, 45, 46\}$.

---

## 2. Ambiente e Dependências Verificadas

* **Sistema Operacional:** Ubuntu Linux (24.04 / ROS 2 Jazzy)
* **Middleware:** ROS 2 Jazzy Jalisco (`/opt/ros/jazzy`)
* **Simulador Físico:** Gazebo Sim `8.11.0` (Harmonic)
* **Pacotes ROS 2:** `rclpy`, `ros_gz_sim`, `ros_gz_bridge`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `teleop_twist_keyboard`
* **Bibliotecas Python:** `python3`, `numpy==1.26.4`, `matplotlib==3.6.3`
* **Compilação de Artigo (opcional):** `pdflatex` (TeX Live)

---

## 3. Estrutura do Repositório

```text
Projeto6_PedroHugo/
├── README.md                          # Este arquivo (instruções de compilação e reprodução)
├── memoria.md                         # Registro de decisões técnicas (ADR), referências e guia de fases
├── fase2_prototipo_2d/                # Protótipos matemáticos didáticos em Python puro (sem ROS/Gazebo)
│   ├── passo1_cinematica_rk2.py       # Euler vs. RK2 e deriva angular em "S"
│   ├── passo2_apf_2d.py               # APF 2D + mínimo local em "U" + GNRON
│   └── passo3_bug2_2d.py              # Bug2 2D (Linha-m, FSM, pontos H_i e L_i)
├── ros2_ws/
│   └── src/projeto6_navegacao/
│       ├── config/bridge_params.yaml  # Mapeamento ros_gz_bridge (/clock, /cmd_vel, /joint_states, /scan, /ground_truth/odom)
│       ├── models/robo_diferencial/   # Modelo físico modular (model.sdf + model.config) com os 4 plugins
│       ├── worlds/
│       │   ├── mundo_etapa1_vazio.sdf # Arena livre (Etapa I)
│       │   ├── cenario_a_esparsos.sdf # Cenário A: obstáculos esparsos e corredor (S=(0,0) -> T=(8,0))
│       │   ├── cenario_b_armadilha_u.sdf # Cenário B: parede côncava em "U" (S=(0,0) -> T=(6,0))
│       │   └── cenario_gnron.sdf      # Subcenário GNRON: parede a 0,45 m atrás da meta (T=(6,0))
│       ├── launch/
│       │   ├── fase3_infraestrutura.launch.py
│       │   ├── etapa1_odometria.launch.py
│       │   └── etapa2_navegacao.launch.py
│       └── projeto6_navegacao/
│           ├── kinematic_model.py     # Funções puras RK2, wrap_to_pi e quatérnios
│           ├── odom_estimator_node.py # Nó Etapa I (/joint_states -> /odom_est)
│           ├── etapa1_test_driver.py  # Gerador das 3 trajetórias da Etapa I
│           ├── metrics_logger_node.py # Avaliador Etapa I (/odom_est vs /ground_truth/odom)
│           ├── apf_navigator_node.py  # Nó Etapa II — Método A (APF + detecção de mínimo local)
│           ├── bug2_navigator_node.py # Nó Etapa II — Método B (Bug2 FSM + H_i/L_i)
│           └── nav_metrics_logger_node.py # Avaliador Etapa II (Tabela 1 completa)
├── scripts/
│   ├── run_etapa1.py                  # Executa as 3 trajetórias da Etapa I (headless) e plota figuras
│   ├── plot_etapa1.py                 # Gera fig1_etapa1_trajetorias.png e fig2_etapa1_erros.png
│   ├── test_fase5_smoke.py            # Validação funcional rápida (1 semente por cenário)
│   ├── run_fase6_experimentos.py      # Bateria completa (30 execuções 5 sementes + 8 paramétricas)
│   └── plot_fase6_resultados.py       # Gera Tabela 1 (MD + LaTeX) e fig3/fig4 da Etapa II
└── resultados/
    ├── etapa1/                        # CSVs e figuras (Fig. 1 e Fig. 2) da Etapa I
    ├── etapa2_bateria/                # 30 execuções (sementes 42..46), Tabela 1 (MD/LaTeX), Fig. 3 e Fig. 4
    └── etapa2_parametrico/            # 8 execuções do estudo paramétrico do APF e limiar GNRON
```

---

## 4. Compilação do Workspace ROS 2

```bash
cd /mnt/Storage/Documentos/Documentos/UFBA/Mestrado/Robotica_Movel/Projeto6_PedroHugo/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 5. Como Reproduzir os Experimentos

> **Lembrete:** Em qualquer terminal novo dentro de `Projeto6_PedroHugo/`, carregue antes o ambiente:
> ```bash
> source /opt/ros/jazzy/setup.bash
> source ros2_ws/install/setup.bash
> ```

### 5.1 Etapa I — Odometria RK2 (Ao Vivo no Gazebo 3D ou Automatizado)

* **Assistir ao vivo no Gazebo 3D** (`trajetoria` pode ser `reta`, `arco` ou `composto`):
  ```bash
  ros2 launch projeto6_navegacao etapa1_odometria.launch.py trajetoria:=composto
  ```
* **Reproduzir automaticamente as 3 trajetórias e gerar as Figuras 1 e 2:**
  ```bash
  python3 scripts/run_etapa1.py
  ```

### 5.2 Etapa II — Demonstração ao Vivo no Gazebo 3D (Para Apresentação Oral)

* **Cenário A (Esparsos e Corredor) — APF vs. Bug2:**
  ```bash
  ros2 launch projeto6_navegacao etapa2_navegacao.launch.py metodo:=apf cenario:=cenario_a
  ros2 launch projeto6_navegacao etapa2_navegacao.launch.py metodo:=bug2 cenario:=cenario_a
  ```
* **Cenário B (Armadilha em "U") — APF trava em mínimo local vs. Bug2 escapa:**
  ```bash
  ros2 launch projeto6_navegacao etapa2_navegacao.launch.py metodo:=apf cenario:=cenario_b
  ros2 launch projeto6_navegacao etapa2_navegacao.launch.py metodo:=bug2 cenario:=cenario_b
  ```
* **Subcenário GNRON (Parede a $0{,}45\text{ m}$ atrás da meta):**
  ```bash
  ros2 launch projeto6_navegacao etapa2_navegacao.launch.py metodo:=apf cenario:=cenario_gnron
  ros2 launch projeto6_navegacao etapa2_navegacao.launch.py metodo:=bug2 cenario:=cenario_gnron
  ```

### 5.3 Bateria Estatística Completa ($N = 5$ sementes) e Estudo Paramétrico do APF

* **Executar todas as 38 rodadas (30 principais + 8 paramétricas) e gerar Tabela 1 e Figuras 3 e 4:**
  ```bash
  python3 scripts/run_fase6_experimentos.py
  ```
* **Replotar figuras e regenerar tabelas a partir dos dados já coletados:**
  ```bash
  python3 scripts/plot_etapa1.py
  python3 scripts/plot_fase6_resultados.py
  ```

### 5.4 Protótipos Didáticos em Python Puro (Fase 2 — Sem ROS/Gazebo)
```bash
cd fase2_prototipo_2d
python3 passo1_cinematica_rk2.py --show
python3 passo2_apf_2d.py --show
python3 passo3_bug2_2d.py --show
```

---

## 6. Principais Resultados Obtidos

* **Etapa I (Odometria RK2 vs. *Ground Truth*):** $\text{RMSE}_p = 3{,}6\text{ mm}$ na linha reta ($5\text{ m}$), $4{,}2\text{ mm}$ ($0{,}23^\circ$) no arco circular ($R_c = 1\text{ m}$) e $3{,}6\text{ mm}$ ($0{,}17^\circ$) no percurso composto em "S".
* **Etapa II ($N = 5$ sementes, `42..46`):**
  * **Cenário A:** Ambos atingiram `100%` de sucesso com `0` colisões; APF foi mais rápido (`33,4 ± 0,1 s` vs. `37,2 ± 0,2 s`), enquanto o Bug2 manteve maior margem de segurança (`d_min = 0,36 m` vs. `0,28 m`).
  * **Cenário B (Armadilha em "U"):** APF falhou em `100%` das execuções (mínimo local interno em $x \approx 2{,}85\text{ m}$), enquanto o Bug2 alcançou `100%` de sucesso (`L = 12,25 ± 0,02 m`, `0` colisões).
  * **Subcenário GNRON:** APF estagnou antes da meta em `100%` das execuções nominais ($\rho_0 = 1{,}10\text{ m}$, erro residual de `0,35 m`), convergindo apenas quando $\rho_0 = 0{,}35\text{ m} < 0{,}45\text{ m}$; o Bug2 atingiu `100%` de sucesso.
