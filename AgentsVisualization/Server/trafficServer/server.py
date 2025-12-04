from traffic_base.agent import *
from traffic_base.model import CityModel
from mesa.visualization import (
    Slider,
    CommandConsole,
    SolaraViz,
    make_space_component,
    make_plot_component,
)
from mesa.visualization.components import AgentPortrayalStyle
import solara


def agent_portrayal(agent):

    if agent is None:
        return

    portrayal = AgentPortrayalStyle(
        marker="s",
    )

    if isinstance(agent, Road):
        portrayal.color = "#aaa"

    if isinstance(agent, Destination):
        portrayal.color = "lightgreen"

    if isinstance(agent, Traffic_Light):
        portrayal.color = "red" if not agent.state else "green"

    if isinstance(agent, Obstacle):
        portrayal.color = "#555"

    if isinstance(agent, Hospital):
        portrayal.color = "purple"

    if isinstance(agent, Ambulance):
        portrayal.color = "orange"

    elif isinstance(agent, CarAgent):
        portrayal.color = "blue"

    return portrayal

model_params = {
    "N": 5,
    "seed": {
        "type": "InputText",
        "value": 42,
        "label": "Random Seed",
    },
    "vehicle_spawn_rate": Slider(
        "Car Spawn Rate", 10, 1, 50, 1,
    ),
    "vehicles_per_step": Slider(
        "Vehicles per step", 4, 0, 4, 1,
    ),
    "ambulance_per_step": Slider(
        "Ambulances per step", 1, 0, 4, 1,
    ),
    "emergency_chance": Slider(
        "Emergency Chance (%)", 0.5, 0, 1, 0.1,
    ),
}

model = CityModel(
    model_params["N"],
    seed=model_params["seed"]["value"],
    vehicle_spawn_rate=model_params["vehicle_spawn_rate"].value,
    vehicles_per_step=model_params["vehicles_per_step"].value,
    ambulance_per_step=model_params["ambulance_per_step"].value,
    emergency_chance=model_params["emergency_chance"].value,
)

# Set equal aspect ratio for the grid
def post_process(ax):
    ax.set_aspect("equal")

# Customize line plot legend
def post_process_lines(ax):
    ax.legend(loc="center left", bbox_to_anchor=(1, 0.9))

# Customize line plot legend with 50 steps window
def post_process_lines_windowed(ax):
    ax.legend(loc="center left", bbox_to_anchor=(1, 0.9))
    # Documentation to get_lines: https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.get_lines
    lines = ax.get_lines()
    # Documentation to get_xdata: https://matplotlib.org/stable/api/_as_gen/matplotlib.lines.Line2D.html#matplotlib.lines.Line2D
    if lines and len(lines[0].get_xdata()) > 50:
        total_steps = len(lines[0].get_xdata())
    # Documentation to set_xlim: https://matplotlib.org/stable/api/_as_gen/matplotlib.axes.Axes.set_xlim.html
        ax.set_xlim(xmin=total_steps - 50, xmax=total_steps)

# Create line plot component to track data over time
lineplot_component = make_plot_component(
    {
        "Total Cars": "tab:blue",
        "Total Ambulances": "tab:red",
        "Emergency Ambulances": "tab:orange",
    },
    post_process=post_process_lines,
)

# Create destinations plot component to track spawned cars and ambulances per step
destinations_component = make_plot_component(
    {
        "Cars Spawned": "tab:blue",
        "Ambulances Spawned": "tab:red"
    },
    post_process=post_process_lines_windowed,
)

# Create destinations plot component to track cars reaching destination and ambulances reaching hospital per step
reached_component = make_plot_component(
    {
        "Cars Reached Destination": "tab:green",
        "Ambulances Reached Hospital": "tab:purple"
    },
    post_process=post_process_lines_windowed,
)

# Create a destination component with total car destination vs cars spawned
cars_destination_component = make_plot_component(
    {
        "Total Cars": "tab:blue",
        "Total Reached Cars Historical": "tab:green",
    },
    post_process=post_process_lines,
)

# Create space component for visualizing the grid
space_component = make_space_component(
    agent_portrayal,
    draw_grid=False,
    post_process=post_process,
)

# Create custom text component for historical statistics
# Documentation for Solara Markdown: https://solara.dev/documentation/components/output/markdown
def StatsDisplay(model):
    return solara.Markdown(f"""
### Historical Statistics

**Total Cars Spawned:** {model.total_cars_historical}

**Total Ambulances Spawned:** {model.total_ambulances_historical}

**Total Cars Reached:** {model.total_reached_cars_historical}

**Total Ambulances Reached:** {model.total_reached_ambulances_historical}

**Total Crashes:** {model.total_crashes}
    """)

# Create Solara visualization page
page = SolaraViz(
    model,
    components=[
        CommandConsole, 
        space_component,
        StatsDisplay,
        lineplot_component,
        destinations_component,
        reached_component,
        cars_destination_component
    ],
    model_params=model_params,
    name="Random Model",
)
