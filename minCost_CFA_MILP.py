from __future__ import print_function
from ortools.linear_solver import pywraplp

def main():
    """Solving Assignment Problem with MIP"""
# Instantiate a mixed-integer solver.
    solver = pywraplp.Solver('SolveAssignmentProblemMIP',
        pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
    #solver = pywraplp.Solver('SolveSimpleSystem',
    #                   pywraplp.Solver.GLOP_LINEAR_PROGRAMMING)

    #Time limit (ms)
    solver.set_time_limit(2000)

    traffic_matrix =    [[0, 12, 15, 23, 10],
                            [12, 0, 2, 7,  3],
                            [15, 2, 0, 4, 5 ],
                            [23, 7, 4, 0, 8 ],
                            [10, 3, 5, 8, 0 ] ]

    linkTable =  [[0, 1],
                        [1, 0],
                        [0, 2],
                        [2, 0],
                        [0, 3],
                        [3, 0],
                        [0, 4],
                        [4, 0],
                        [1, 2],
                        [2, 1],
                        [2, 3],
                        [3, 2],
                        [3, 4],
                        [4, 3],
                        [4, 1],
                        [1, 4]]

    num_nodes = len(traffic_matrix)
    num_flows =  num_nodes*num_nodes
    num_links = len(linkTable)

    #node_link_incidence_matrix (node_id, link_id)   --> topology   3-node ring : 6 unidirectional links
    #link-id = (s_node-1)*N+d_node
    incidence_matrix = [[0 for ij in range(num_links)] for n in range(num_nodes)]
    for ij in range(num_links):
        a=linkTable[ij][0]
        b=linkTable[ij][1]
        incidence_matrix[a][ij]=1
        incidence_matrix[b][ij]=-1
    capacity = 50
    cost =  1

    x = {}
    for sd in range(num_flows):
        for ij in range(num_links):
            x[sd, ij] = solver.NumVar(0.0, solver.infinity(), 'x[%sd,%ij]' % (sd, ij))

    y = {}
    for ij in range(num_links):
        #y[ij] = solver.NumVar(0.0, solver.infinity(),'y[%ij]' % ij)
        y[ij] = solver.IntVar(0.0, solver.infinity(),'y[%ij]' % ij)

  # Objective
    #solver.Minimize(solver.Sum([cost * x[sd, ij]  for sd in range(num_flows) for ij in range(num_links)]))
    solver.Minimize(solver.Sum([cost * y[ij]  for ij in range(num_links)]))

  # Constraints
  # Flow conservation ctr:
    for s in range(num_nodes):
        for d in range(num_nodes):
            sd =  (s)*num_nodes+d
            for n in range(num_nodes):
                if n == s: # Source node
                    solver.Add(solver.Sum([incidence_matrix[n][ij]* x[sd, ij] for ij in range(num_links)]) == traffic_matrix[s][d])
                elif n== d: # Destination node
                    solver.Add(solver.Sum([incidence_matrix[n][ij]* x[sd, ij] for ij in range(num_links)]) == -traffic_matrix[s][d])
                else: # Intermediate node
                    solver.Add(solver.Sum([incidence_matrix[n][ij]* x[sd, ij] for ij in range(num_links)]) == 0)

  # Capacity constraints
    capacityCtrs = [0] * num_links
    for ij in range(num_links):
        '''
        for every pair in the link matrix, the capacity y for every link is larger than
        the traffic x going through it
        this leads to the constraint [x[sd, ij] - capacity * y[ij] <=0
        '''
        capacityCtrs[ij] = solver.Constraint(-solver.infinity(), 0)
#        capacityCtrs[ij].SetCoefficient(y[ij], -1)
        capacityCtrs[ij].SetCoefficient(y[ij], -capacity)
        for sd in range(num_flows):
            capacityCtrs[ij].SetCoefficient(x[sd, ij], 1)
#    for ij in range(num_links):
#        solver.Add(solver.Sum([x[sd, ij] for sd in range(num_flows)]) <= capacity)


    #   Calling Solver
    status = solver.Solve()
    if status == solver.OPTIMAL:
        print ('A  optimal solution was found.')
    else:  # No optimal solution was found.
        if status == solver.FEASIBLE:
            print ('A potentially suboptimal solution was found.')
        else:
              print ('The solver could not solve the problem.')
              #return
    #print('Total cost = ', solver.Objective().Value())
    print('Optimal Objective Value = ', solver.Objective().Value())
    print('Number of variables =', solver.NumVariables())
    print('Number of constraints =', solver.NumConstraints())
    print("Time = ", solver.WallTime(), " milliseconds")
    print("No. of simplex iterations = ", solver.iterations(), "")

    print()

    for sd in range(num_flows):
        for ij in range(num_links):
            if x[sd, ij].solution_value() > 0:
                print('Flow %d assigned to link %d  Flow = %d' % (
                sd,
                ij,
                x[sd, ij].solution_value()))

    print()

    for ij in range(num_links):
        if y[ij].solution_value() > 0:
                print('Capacity assigned to link %d   Num. capacity modules= %f' % (
                ij,
                y[ij].solution_value()))
    print()

    '''
    # Only for LP
    for ij in range(num_links):
        print('Link %d   Dual value (price) = %f' % (
        ij,
        capacityCtrs[ij].dual_value()))
    print()
    for ij in range(num_links):
        if y[ij].solution_value() > 0:
                print('Capacity assigned to link %d   Reduced cost= %f  Basis status= %f' % (
                ij,
                y[ij].reduced_cost(),
                y[ij].basis_status()))
    '''
#   Advanced usage: possible basis status values for a variable and the slack variable of a linear constraint.
# enum BasisStatus {
#      FREE = 0,
#      AT_LOWER_BOUND,
#      AT_UPPER_BOUND,
#      FIXED_VALUE,
#      BASIC
#    };
#
    print()

    # Only for ILP
    print("No. of branch-and-bound nodes = ", solver.nodes(), "")
    print("Best objective bound = ", solver.Objective().BestBound(), "")




if __name__ == '__main__':
    main()
