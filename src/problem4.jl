using Concorde

opt_tour, opt_len = solve_tsp("./data/att48.tsp")
# Optimal length: 10628
println("Optimal tour: ", opt_tour)
println("Optimal length: ", opt_len)

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
#     @show typeof(X)
#     @show typeof(Y)

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

# X, Y = solve_tsp_q()
# opt_tour, opt_len = solve_tsp(X, Y; dist="EUC_2D")

# 33522