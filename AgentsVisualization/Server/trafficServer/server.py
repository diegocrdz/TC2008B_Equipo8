from traffic_base.agent import *
from traffic_base.model import CityModel
from mesa.visualization import (
    CommandConsole,
    Slider,
    SolaraViz,
    SpaceRenderer,
    make_space_component,
)
from mesa.visualization.components import AgentPortrayalStyle


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

    if isinstance(agent, Ambulance):
        portrayal.color = "orange"

    if (isinstance(agent, CarAgent)):
        portrayal.color = "blue"
    
    if isinstance(agent, Hospital):
        portrayal.color = "purple"

    return portrayal

model_params = {
    "N": 5,
    "seed": {
        "type": "InputText",
        "value": 42,
        "label": "Random Seed",
    },
    "car_spawn_rate": Slider(
        "Car Spawn Rate", 10, 0, 10, 1,
    ),
    "vehicles_per_step": Slider(
        "Vehicles per step", 1, 0, 4, 1,
    ),
    "ambulance_per_step": Slider(
        "Ambulances per step", 1, 0, 4, 1,
    ),
}

model = CityModel(
    model_params["N"],
    seed=model_params["seed"]["value"],
    car_spawn_rate=model_params["car_spawn_rate"].value,
    vehicles_per_step=model_params["vehicles_per_step"].value,
    ambulance_per_step=model_params["ambulance_per_step"].value,
)

def post_process(ax):
    ax.set_aspect("equal")

space_component = make_space_component(
    agent_portrayal,
    draw_grid=False,
    post_process=post_process,
)

page = SolaraViz(
    model,
    components=[CommandConsole, space_component],
    model_params=model_params,
    name="Random Model",
)
