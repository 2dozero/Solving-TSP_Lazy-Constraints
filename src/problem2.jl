using JuMP, GLPK, Distances
using Gurobi
using BenchmarkTools

function solve_tsp(;file_name="st70.tsp", benchmark=false)
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
    close(f)

    # Extract X and Y coordinates
    # @show coords
    X = [coord[1] for coord in coords]
    Y = [coord[2] for coord in coords]
    # @show X
    # @show Y

    # Create distance matrix
    N = length(coords)
    d = zeros(N, N)
    for i in 1:N
        for j in i+1:N
            d[i, j] = sqrt((X[i] - X[j])^2 + (Y[i] - Y[j])^2)
            d[j, i] = d[i, j]  # The distance matrix is symmetric
        end
    end
    m = Model(Gurobi.Optimizer)
    @variable(m, x[1:N,1:N], Bin)
    @variable(m, t[1:N], Int)

    # @objective(m, Min, sum(x[i,j]*dist_mat[i,j] for i=1:N,j=1:N))
    @objective(m, Min, sum(d .* x))

    for i=1:N 
        @constraint(m, x[i,i] == 0)
        @constraint(m, sum(x[i,1:N]) == 1)
    end
    for j=1:N
        @constraint(m, sum(x[1:N,j]) == 1)
    end
    # for i in 1:N
    #     @constraint(m, sum(x[i, :])==1)
    #     @constraint(m, sum(x[:, i])==1)
    # end
    # @constraint(m, [i in 1:N], x[i, i] == 0)


    # for f=1:N, t=1:N
    #     @constraint(m, x[f,t]+x[t,f] <= 1)
    # end
    @constraint(m, t .>= 0)
    for i=2:N, j=2:N
        if i != j
            @constraint(m, t[j] >= t[i] + N * x[i, j] - (N - 1)) 
        end
    end

    return m
end

elapsed_time = @elapsed begin
    model_ = solve_tsp()
    optimize!(model_)
end

# 최적화 완료 후 결과 출력
println("Objective value: ", JuMP.objective_value(model_))
println("Solve TSP execution time: ", elapsed_time, " seconds")
# model_ = solve_tsp()
# optimize!(model_)

# optimize!(m)
# !benchmark && println("Obj: ", JuMP.objective_value(m))

# if JuMP.termination_status(m) == MOI.OPTIMAL
#     optimal = 1
# else
#     optimal = 0
# end
# if !benchmark
#     println("Optimal: ", optimal)
#     println("Termination status: ", JuMP.termination_status(m))
# end

# x_val = value.(x)
# tour = []
# for i in 1:N
#     for j in 1:N
#         if x_val[i, j] > 0.5
#             push!(tour, (i, j))
#         end
#     end
# end