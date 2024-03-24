using Concorde

elapsed_time = @elapsed begin
    opt_tour, opt_len = Concorde.solve_tsp("./data/st70.tsp")
end

# Optimal length: 10628
println("Optimal tour: ", opt_tour)
println("Optimal length: ", opt_len)
println("Solve TSP execution time: ", elapsed_time, " seconds")

# using Concorde

# function solve_tsp_q(;file_name="att48.tsp", benchmark=false)
#     coords = []
#     start_reading = false
#     f = open("./data/" * file_name)
#     for line in eachline(f)
#         if start_reading
#             if line == "EOF"
#                 break
#             end
#             _, x, y = split(line)
#             push!(coords, (parse(Float64, x), parse(Float64, y)))
#         elseif line == "NODE_COORD_SECTION"
#             start_reading = true
#         end
#     end
#     close(f)

#     # Extract X and Y coordinates
#     # @show coords
#     X = [coord[1] for coord in coords]
#     Y = [coord[2] for coord in coords]
#     # @show X
#     # @show Y
#     # @show typeof(X)
#     # @show typeof(Y)

#     # # Create distance matrix
#     # N = length(coords)
#     # d = zeros(N, N)
#     # for i in 1:N
#     #     for j in i+1:N
#     #         d[i, j] = sqrt((X[i] - X[j])^2 + (Y[i] - Y[j])^2)
#     #         d[j, i] = d[i, j]  # The distance matrix is symmetric
#     #     end
#     # end
#     return X, Y
# end

# x, y = solve_tsp_q()
# opt_tour, opt_len = Concorde.solve_tsp(x, y; dist="EUC_2D")