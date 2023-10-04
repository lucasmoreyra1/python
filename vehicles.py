import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from fastapi import FastAPI
from pydantic import BaseModel
from typing import List




app = FastAPI()


class DistanceMatrixRequest(BaseModel):
    matrix: List[List[float]]  # Cambia a List[List[float]]
    numVehicles: int

# [START data_model]
def create_data_model(distances, num_vehicles):
    """Stores the data for the problem."""
    data = {}
    decimal_matrix = distances.matrix  # Obtén la matriz de distancias como números decimales
    num_points = len(decimal_matrix)
    
    # Redondea los valores decimales a enteros
    integer_matrix = [[int(value) for value in row] for row in decimal_matrix]
    data["distance_matrix"] = integer_matrix  # Almacena la matriz de enteros
    
    # Calculate the capacity per vehicle
    capacity_per_vehicle = num_points // num_vehicles
    vehicle_capacities = [capacity_per_vehicle] * num_vehicles
    
    # If there are remaining points, distribute them among the vehicles
    remaining_capacity = num_points % num_vehicles
    for i in range(remaining_capacity):
        vehicle_capacities[i] += 1
    
    data["demands"] = [0] + [1] * (num_points - 1)  # Assuming the first point is the depot
    data["vehicle_capacities"] = vehicle_capacities
    data["num_vehicles"] = num_vehicles
    data["depot"] = 0
    
    return data
    # [END data_model]

@app.post('/')
async def calculate_routes(distances: DistanceMatrixRequest):
    numVehicles = distances.numVehicles
    data = create_data_model(distances, numVehicles)


    # Create the routing index manager.
    # [START index_manager]
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )
    # [END index_manager]

    # Create Routing Model.
    # [START routing_model]
    routing = pywrapcp.RoutingModel(manager)
    # [END routing_model]

    # Create and register a transit callback.
    # [START transit_callback]
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    # [END transit_callback]

    # Define cost of each arc.
    # [START arc_cost]
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    # [END arc_cost]

    # Add Capacity constraint.
    # [START capacity_constraint]
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity",
    )
    # [END capacity_constraint]

    # Setting first solution heuristic.
    # [START parameters]
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )
    search_parameters.local_search_metaheuristic = (
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    )
    search_parameters.time_limit.FromSeconds(1)
    # [END parameters]

    # Solve the problem.
    # [START solve]
    solution = routing.SolveWithParameters(search_parameters)
    # [END solve]

    # Print solution on console.
    # [START print_solution]
    if solution:
        return get_solution_as_array(manager, routing, solution)
    # [END print_solution]

def get_solution_as_array(manager, routing, solution):
    all_routes = []
    for vehicle_id in range(manager.GetNumberOfVehicles()):
        index = routing.Start(vehicle_id)
        route = [manager.IndexToNode(index)]
        while not routing.IsEnd(index):
            index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))
        all_routes.append(route)
    return all_routes

