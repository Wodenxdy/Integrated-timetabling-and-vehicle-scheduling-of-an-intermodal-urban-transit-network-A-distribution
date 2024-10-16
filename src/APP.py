from Gurobi_DRT_Group import GurobiModel

if __name__ == "__main__":
    MyModel = GurobiModel()
    MyModel.add_variables()
    MyModel.add_constraints_timetable()
    MyModel.add_constraints_passengers_movement()

    MyModel.add_constraints_passenger_FIX()
    MyModel.add_constraints_DRT(disable=False)
    MyModel.add_constraints_vehicle_circulation()
    MyModel.add_objective_function()
    MyModel.add_DRO_constraints()

    MyModel.solve_model()
