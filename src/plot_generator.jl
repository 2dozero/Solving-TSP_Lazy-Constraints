using JuMP, GLPK, Distances
using UnicodePlots, Logging, LinearAlgebra, Printf
import MathOptInterface
using Gurobi
using TravelingSalesmanExact
using Plots
import Plots: savefig

# Solve the TSP instance as an integer programming problem with subtour elimination constraints added as lazy constraints using either Gurobi or CPLEX.

function is_tsp_solved(m, x; benchmark=false, coords)
    N = size(x)[1] # N : 48
    x_val = JuMP.value.(x) # x_val : [0.0, -0.0, 1.0 .... ]
    
    # find cycle
    cycle_idx = Int[]
    push!(cycle_idx, 1) # cycle_idx : [1]
    while true
        v, idx = findmax(x_val[cycle_idx[end],1:N]) # idx = (1.0, 6)
        if idx == cycle_idx[1]
            break
        else
            push!(cycle_idx,idx)
        end
    end
    if !benchmark
        println("cycle_idx: ", cycle_idx)
        println("Length: ", length(cycle_idx))
    end
    
    if length(cycle_idx) < N
        @constraint(m, sum(x[cycle_idx,cycle_idx]) <= length(cycle_idx) - 1) # cycle_dix : [1, 6, 3, 7]
        return false
    end
    return true
end 

function solve_tsp(;file_name="att48.tsp", benchmark=false)
    # f = open("./data/" * file_name);
    # lines = readlines(f)
    coords = []
    start_reading = false
    f = open("./data/" * file_name)
        for line in eachline(f)
            if start_reading
                if line == "EOF"
                    break
                end
                _, x, y = split(line)
                push!(coords, (parse(Float64, x), parse(Float64, y)))
            elseif line == "NODE_COORD_SECTION"
                start_reading = true
            end
        end

    N = length(coords)
    !benchmark && println("N: ", N)

    m = Model(Gurobi.Optimizer)
    dist_mat = zeros(N,N)
    for i=1:N, j=i+1:N
        d = euclidean(coords[i], coords[j])
        dist_mat[i,j] = d
        dist_mat[j,i] = d
    end
    @variable(m, x[1:N,1:N], Bin)
    @objective(m, Min, sum(x[i,j]*dist_mat[i,j] for i=1:N,j=1:N))
    for i=1:N 
        @constraint(m, x[i,i] == 0)
        @constraint(m, sum(x[i,1:N]) == 1)
    end
    for j=1:N
        @constraint(m, sum(x[1:N,j]) == 1)
    end
    for f=1:N, t=1:N
        @constraint(m, x[f,t]+x[t,f] <= 1)
    end

    memory = []
    optimize!(m)
    x_val = JuMP.value.(x)
    push!(memory, x_val)
    !benchmark && println("Obj: ", JuMP.objective_value(m))

    while !is_tsp_solved(m, x; benchmark=benchmark, coords)
        optimize!(m)
        x_val = JuMP.value.(x)
        push!(memory, x_val)
    end

    !benchmark && println("Obj: ", JuMP.objective_value(m))

    if JuMP.termination_status(m) == MOI.OPTIMAL
        optimal = 1
    else
        optimal = 0
    end
    if !benchmark
        println("Optimal: ", optimal)
        println("Termination status: ", JuMP.termination_status(m))
    end

    x_coords = [coord[1] for coord in coords]
    y_coords = [coord[2] for coord in coords]
    for i in 1:length(memory)
        generate_and_save_plots(x_coords, y_coords, memory[i], "plot_$i.png")
    end


end

# function plot_tour(cities, perm_matrix)
#     cycles = get_cycles(perm_matrix)
#     tour = reduce(vcat, cycles)
#     return plot_cities(cities[tour])
# end

function plot_cities(cities)
    loop = [cities[mod1(j+1, end)] for j = 0:length(cities)]
    return lineplot(first.(loop), last.(loop); height=18)
end

function get_cycles(perm_matrix)
    N = size(perm_matrix, 1)
    remaining_inds = Set(1:N)
    cycles = Vector{Int}[]
    while length(remaining_inds) > 0
        cycle = find_cycle(perm_matrix, first(remaining_inds))
        push!(cycles, cycle)
        setdiff!(remaining_inds, cycle)
    end
    return cycles
end

function find_cycle(perm_matrix, starting_ind = 1)
    cycle = [starting_ind]
    prev_ind = ind = starting_ind
    while true
        # the comparisons `x > (0.5)` should mean `x == 1`. Due to floating point results returned
        # by the solvers, instead we sometimes have `x ≈ 1.0` instead. Since these are binary
        # values, we might as well just compare to 1/2.
        next_ind = findfirst(>(0.5), @views(perm_matrix[ind, 1:prev_ind-1]))
        if isnothing(next_ind)
            next_ind = findfirst(>(0.5), @views(perm_matrix[ind, prev_ind+1:end])) +
                       prev_ind
        end
        next_ind == starting_ind && break
        push!(cycle, next_ind)
        prev_ind, ind = ind, next_ind
    end
    return cycle
end

function main()
    solve_tsp()
end

function selected_edges(x, N)
    return [(i, j) for i in 1:N for j in 1:N if x[i, j] > 0.5]
end

function plot_tour(X, Y, x)
    plot = Plots.plot()
    for (i, j) in selected_edges(x, size(x, 1))
        Plots.plot!([X[i], X[j]], [Y[i], Y[j]]; legend = false)
    end
    return plot
end


function generate_and_save_plots(X, Y, edges, filename)
    plot = plot_tour(X, Y, edges)
    # savefig(plot, filename)
    savefig(plot, "./image/" * filename)
end


# # plot_tour(coords[:, ], Y, memory[3])

# function generate_and_save_plots(X, Y, edges, filename)
#     plot = plot_tour(X, Y, edges)
#     savefig(plot, filename)
# end
# for i in 1:length(optimal_edges)
#     generate_and_save_plots(X, Y, optimal_edges[i], "plot_$i.png")
# end


main()