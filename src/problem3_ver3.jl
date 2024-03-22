using JuMP, Gurobi

function solve_TSP_lazy(x_c::Vector{Float64}, y_c::Vector{Float64})
    N = size(x_c)[1]
    m = Model(Gurobi.Optimizer)
    dist_mat = zeros(N,N)
    for i=1:N, j=1:N
        dist_mat[i, j] = sqrt((x_c[i]-x_c[j])^2 + (y_c[i]-y_c[j])^2)
    end
    @variable(m, x[1:N,1:N], Bin)
    @objective(m, Min, sum(x[i,j] * dist_mat[i,j] for i=1:N,j=1:N))
    for i=1:N 
        @constraint(m, x[i,i] == 0)
        @constraint(m, sum(x[i,1:N]) == 1)
    end
    for i=1:N, j=1:N
        @constraint(m, 0 <= x[i, j] <= 1)
    end
    for j=1:N
        @constraint(m, sum(x[1:N,j]) == 1)
    end
    # for f=1:N, t=1:N
    #     @constraint(m, x[f,t]+x[t,f] <= 1)
    # end

    function call_back_function(cb_data)
        x_val = (y -> callback_value(cb_data, y)).(x)

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
        if length(cycle_idx) < N
            con = @build_constraint(sum(x[cycle_idx,cycle_idx]) <= length(cycle_idx) - 1) # cycle_dix : [1, 6, 3, 7]
            MOI.submit(m, MOI.LazyConstraint(cb_data), con)
        end
    end
    # Register the above call_back_function() to the TSP model .
    MOI.set(m, MOI.LazyConstraintCallback(), call_back_function)

    optimize!(m)

    println("optimization is done.")
    x_val = value.(x)

    # opt_tour = cycle_idx
    opt_len = objective_value(m)

    return opt_len

end



function read_tsp_file(;file_name="att48.tsp", benchmark=false)
    start_reading = false
    f = open("./data/" * file_name)
    x_coords = Float64[]
    y_coords = Float64[]
    for line in eachline(f)
        if start_reading
            if line == "EOF"
                break
            end
            _, x, y = split(line)
            push!(x_coords, parse(Float64, x))
            push!(y_coords, parse(Float64, y))
        elseif line == "NODE_COORD_SECTION"
            start_reading = true
        end
    end

    return x_coords, y_coords
    # @show x_coords, y_coords
end

# x, y = read_tsp_file()
# solve by lazy constraints and Gurobi
# generate a random instance
n = 20
x = rand(n)
y = rand(n)
lazy_tour_len = solve_TSP_lazy(x, y)

# solve by the Concorde solver
# length_concorde = solve_TSP_Concorde(x, y)

# @show lazy_tour
# @show tour_concorde
# @show lazy_tour_len, length_concorde
# @assert isapprox(lazy_tour_len, length_concorde)
println("lazy_tour_len: ", lazy_tour_len)
println("done")