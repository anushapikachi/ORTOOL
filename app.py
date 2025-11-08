from flask import Flask, request, jsonify
from flask_cors import CORS
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

app = Flask(__name__)
CORS(app)

@app.route('/optimize', methods=['POST'])
def optimize():
    data = request.get_json()

    distance_matrix = data["distance_matrix"]
    depot = data["depot"]
    num_vehicles = data["num_vehicles"]
    vehicle_capacities = data["vehicle_capacities"]
    demands = data["demands"]
    return_to_depot = data.get("return_to_depot", True)

    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), num_vehicles, depot)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return demands[from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(demand_callback_index, 0, vehicle_capacities, True, "Capacity")

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_parameters)
    routes = {}

    if solution:
        for vehicle_id in range(num_vehicles):
            index = routing.Start(vehicle_id)
            route = []
            while not routing.IsEnd(index):
                route.append(manager.IndexToNode(index))
                index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))
            routes[f"driver{vehicle_id+1}"] = route

        if not return_to_depot:
            for key in routes:
                if routes[key][-1] == depot:
                    routes[key].pop()
    else:
        routes = {"error": "No solution found"}

    return jsonify(routes)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=10000)
