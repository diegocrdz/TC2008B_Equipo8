# Guía de Instalación y Ejecución - TC2008B Equipo 8

### 1. Clonar el Repositorio

```bash
git clone git@github.com:diegocrdz/TC2008B_Equipo8.git
cd TC2008B_Equipo8
```

### 2. Crear Entorno Virtual

**Windows:**
```bash
python -m venv .agents
source .agents/Scripts/activate
```

**Linux/Mac:**
```bash
python3 -m venv .agents
source .agents/bin/activate
```

### 3. Instalar Dependencias de Python

```bash
pip install -U mesa[all]
pip install flask flask-cors
```

### 4. Instalar Dependencias de Node.js (para la visualización 3D)

```bash
cd AgentsVisualization
npm install
```

---

## Ejecución de la Simulación

### Opción 1: Simulación en 2D (Solara)

```bash
cd AgentsVisualization/Server/trafficServer
solara run server.py
```

### Opción 2: Simulación en 3D (Vite + Flask)

**Terminal 1 - Servidor Flask (Backend):**
```bash
cd AgentsVisualization/Server/trafficServer
python traffic_server.py
```

**Terminal 2 - Servidor Vite (Frontend):**
```bash
cd AgentsVisualization
npx vite
```
