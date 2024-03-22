using JuMP, Distances
using UnicodePlots, Logging, LinearAlgebra, Printf
import MathOptInterface
const MOI = MathOptInterface
using Gurobi
export get_optimal_tour,
       plot_cities,
       simple_parse_tsp,
       set_default_optimizer!

const default_optimizer = Ref{Any}(nothing)
const SLOW_SLEEP = Ref(1.5)
set_default_optimizer!(O) = default_optimizer[] = Of
get_default_optimizer() = default_optimizer[]
reset_default_optimizer!() = default_optimizer[] = nothing

function remove_cycles!(model, tour_matrix; symmetric = true)
    tour_matrix_val = value.(tour_matrix)
    cycles = get_cycles(tour_matrix_val)
    length(cycles) == 1 && return 1
    for cycle in cycles
        constr = symmetric ? 2 * length(cycle) - 2 : length(cycle) - 1
        @constraint(model, sum(tour_matrix[cycle, cycle]) <= constr)
    end
    return length(cycles)
end

function solve_tsp(;file_name="att48.tsp", benchmark=false, verbose = true, slow = true, lazy_constraints = true)
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
    # !benchmark && println("N: ", N)

    m = Model(Gurobi.Optimizer)
    dist_mat = zeros(N,N)
    for i=1:N, j=i+1:N
        d = euclidean(coords[i][1], coords[j][2])
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

    has_cities = !isnothing(coords)
    if has_cities && verbose
        @info "Starting optimization." plot_cities(coords)
    elseif verbose
        @info "Starting optimization."
    end

    # count for logging
    iter = Ref(0)
    tot_cycles = Ref(0)
    all_time = Ref(0.0)

    if lazy_constraints
        remove_cycles_callback = make_remove_cycles_callback(m, x, has_cities, coords, verbose, true, tot_cycles)
        MOI.set(m, MOI.LazyConstraintCallback(), remove_cycles_callback)
    end

    num_cycles = 2 # just something > 1
    while num_cycles > 1
        t = @elapsed optimize!(m)
        all_time[] += t
        status = termination_status(m)
        status == MOI.OPTIMAL || @warn("Problem status not optimal; got status $status")
        num_cycles = remove_cycles!(m, x; symmetric = true)
        tot_cycles[] += num_cycles
        iter[] += 1
        if verbose
            if num_cycles == 1
                description = "found a full cycle!"
            else
                description = "disallowed $num_cycles cycles."
            end

            if has_cities
                slow && sleep(max(0, SLOW_SLEEP[] - t))
                @info "Iteration $(iter[]) took $(format_time(t)), $description" plot_tour(
                    coords,
                    value.(x),
                )
            else
                @info "Iteration $(iter[]) took $(format_time(t)), $description"
            end
        end
    end
    tot_cycles[] -= 1 # remove the true cycle

    status = termination_status(m)
    status == MOI.OPTIMAL || @warn(status)

    cycles = get_cycles(value.(x))
    length(cycles) == 1 || error("Something went wrong; did not elimate all subtours. Please file an issue.")

    if verbose
        slow && sleep(SLOW_SLEEP[])
        obj = objective_value(m)
        obj_string = isinteger(obj) ? @sprintf("%i", obj) : @sprintf("%.2f", obj)
        @info "Optimization finished; adaptively disallowed $(tot_cycles[]) cycles."
        @info "The optimization runs took $(format_time(all_time[])) in total."
        @info "Final path has length $(obj_string)."
        @info "Final problem has $(num_constraints(model, VariableRef, MOI.ZeroOne)) binary variables,
            $(num_constraints(m,
            GenericAffExpr{Float64,VariableRef}, MOI.LessThan{Float64})) inequality constraints, and
            $(num_constraints(m, GenericAffExpr{Float64,VariableRef}, MOI.EqualTo{Float64})) equality constraints."
    end
    return first(cycles), objective_value(m)
end

function make_remove_cycles_callback(model, tour_matrix, has_cities, cities, verbose, symmetric::Bool, tot_cycles)
    num_triggers = Ref(0)
    return function remove_cycles_callback(cb_data)
        tour_matrix_val = callback_value.(Ref(cb_data), tour_matrix)

        # We only handle integral solutions
        # Could possibly also find cycles in the integer part of a mixed solution.
        any(x -> !(x ≈ round(Int, x)), tour_matrix_val) && return

        num_triggers[] += 1
        cycles = get_cycles(tour_matrix_val)

        if length(cycles) == 1
            if has_cities && verbose
                @info "Lazy constaint triggered ($(num_triggers[])); found a full cycle!" plot_tour(
                    cities,
                    tour_matrix_val,
                )
            elseif verbose
                @info "Lazy constaint triggered ($(num_triggers[])); found a full cycle!"
            end
            return nothing
        end

        for cycle in cycles
            constr = symmetric ? 2 * length(cycle) - 2 : length(cycle) - 1
            cycle_constraint = @build_constraint( sum(tour_matrix[cycle, cycle]) <= constr)
            MOI.submit(model, MOI.LazyConstraint(cb_data), cycle_constraint)
        end

        num_cycles = length(cycles)
        tot_cycles[] += num_cycles
        if has_cities && verbose
            @info "Lazy constaint triggered ($(num_triggers[])); disallowed $num_cycles cycles." plot_tour(
                cities,
                tour_matrix_val,
            )
        elseif verbose
            @info "Lazy constaint triggered ($(num_triggers[])); disallowed $num_cycles cycles."
        end
    end
    return nothing
end

function format_time(t)
    # I want more decimal digits to print the smaller the number is,
    # but never more than 4 decimal digits.
    # I think `round(t; sigdigits = ...)` might be a way to do
    # something like this, but I couldn't get it to work.
    str = if t > 100
        @sprintf("%.0f", t)
    elseif t > 1
        @sprintf("%.2f", t)
    elseif t > 0.1
        @sprintf("%.3f", t)
    else
        @sprintf("%.4f", t)
    end
    return str * " seconds"
end

function plot_tour(cities, perm_matrix)
    cycles = get_cycles(perm_matrix)
    tour = reduce(vcat, cycles)
    return plot_cities(cities[tour])
end

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

main()