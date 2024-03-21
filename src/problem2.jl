using JuMP, GLPK, Distances
using Gurobi

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
        d = euclidean(coords[i][1], coords[j][2])
        dist_mat[i,j] = d
        dist_mat[j,i] = d
    end
    @variable(m, x[1:N,1:N], Bin)
    @variable(m, t[1:N])

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
    @constraint(m, t .>= 0)
    for i=2:N, j=2:N
        if i != j
            @constraint(m, t[j] >= t[i] + N * x[i, j] - (N - 1)) 
        end
    end
    optimize!(m)
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
    # OPTIMAL tour
    x_val = JuMP.value.(x)
    cycle_idx = Int[]
    push!(cycle_idx, 1)
    while true
        v, idx = findmax(x_val[cycle_idx[end], 1:N])
        if idx == cycle_idx[1]
            push!(cycle_idx, idx)
            break
        else
            push!(cycle_idx, idx)
        end
    end
    @show cycle_idx
end

solve_tsp()