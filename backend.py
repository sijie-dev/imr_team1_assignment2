import copy
from model_uos_imr import graphslam_backend
from model_uos_imr import graphslam_frontend

def optimise(graph,
             residual_threshold=1E-4,   
             delta_threshold=1/1000,
             lim_iterations=20,
             verbose=False):            
    graph_opt = copy.deepcopy(graph)

    graph_opt.construct_graph()

    n_iterations = 0
    delta_residual = 100
    residual = 100

    iteration_continue = True
    residual_continue = True
    converge_continue = True
    visualise_flag = False

    # pose_graph = graph_opt.reduce2pose(visualise_flag)

    while iteration_continue and residual_continue and converge_continue:

        backend = graphslam_backend(graph_opt)
        backend.solve()

        previous_residual = residual
        residual = backend.residual
        delta_residual = abs((previous_residual - residual) / previous_residual)
        n_iterations += 1

        residual_continue = (residual > residual_threshold)
        iteration_continue = (n_iterations < lim_iterations)
        converge_continue = (delta_residual > delta_threshold)

        if verbose:
            print("iteration %d: residual %.2e, relative change %.2e"
                  % (n_iterations, residual, delta_residual))

        graph_opt = graphslam_frontend(backend)
        graph_opt.construct_graph()

        # pose_graph = graph_opt.reduce2pose(visualise_flag)

    if verbose:
        print("optimise: %d iterations, final residual %.2e" % (n_iterations, residual))

    return graph_opt