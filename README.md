# Solving TSP with Lazy Constraints

The symmetric Traveling Salesman Problem (TSP) instances (such as berlin52.tsp, st70.tsp, etc.) were addressed using Lazy Constraints for subtour elimination. This was implemented through two different methods. 

![TSPberlin52](gif/berlin52_output.gif)
![TSPst70](gif/st70_output.gif)

## Experiments
![Table](image/Experiments.png)
| TSP      | All SE          |                | Iterative SE   |                | SE by Lazy     |                | Concorde       |  
|          | Tour Length     | Time(s)        | Tour Length    | Time(s)        | Tour Length    | Time(s)        | Tour Length    | Time(s)        |
|:---------|----------------:|---------------:|---------------:|---------------:|---------------:|---------------:|---------------:|---------------:|
| berlin25 | 5,460           | 1.692          | 5,459          | 1.081          | 5,459          | 0.306          | 5,459          | 0.122          |
| berlin52 | 7,544           | 1.032          | 7,542          | 0.295          | 7,542          | 4.573          | 7,542          | 0.025          |
| st70     | 677             | 178.502        | 675            | 1750.761       | 675            | 83.187         | 675            | 0.039          |
| bier127  | -               | -              | -              | -              | -              | -              | 118,282        | 0.157          |

In the experiments, it was confirmed that among all methods, using the Concorde solver resulted
in the fastest derivation of the optimal solution. Considering the time it took other methods to
solve st70.tsp, which consists of 70 nodes, it was evident why Concorde is considered an optimizer
tailored for TSP.

In the case of Iterative SE for berlin52.tsp, it operated faster than the lazy constraint approach,
leading to expectations of good performance on st70 as well. However, possibly due to the repetitive
nature of model optimization, it was observed that the speed significantly slows down once surpassing
a certain number of nodes.
Given that the problems we aim to solve must be of a realistic size, taking such factors into
account, the speed of the models can be ranked as follows: Concorde > SE by Lazy > All SE >
Iterative SE.

## Code Explanation
In 'Iterative_SE.jl'(version 1), the code optimizes the TSP and checks if it's solved by using the Is_Tsp_Solved function. If the problem is solved, the objective value is printed, and the program exits. If not, constraints related to it are iteratively added at each step. The process of checking if it’s solved involves storing decision variables with x-values in cycle_idx, and if its length is less than the number of nodes, it indicates that a subtour still exists.

In 'SE_with_Lazy'(version 2), as opposed to version 1, a callback function is created within the solve_TSP_lazy function. This method utilizes the LazyConstraintCallback() approach on the model, which pre-informs the solver to execute callbacks via the callback function whenever a new Integer solution is found, during which it enforces @build constraints. The method for determining the presence of subtours within the callback function is structured the same as in version 1.

It can be considered that versions 1 and 2 essentially perform the callback operation in a similar manner. However, unlike version 1, which requires multiple executions of the model optimization, version 2 carries out the model optimization process only once. Thus, it can be inferred that the execution speed of version 1 will be slower than that of version 2.

### berlin52.tsp solution
![berlin52_solution](image/berlin52/plot_6.png)
### st70.tsp solution


![st70_solution](image/st70/plot_394.png)

If you want to create a plot for the TSP, you can use 'plot_generator.jl'. After that, you can generate a gif file through 'gif.py'.


## Acknowledgements
This repository includes adaptions of the following repositories as baselines:
* [tsp_lazy_template] 
https://gist.github.com/chkwon/d10d0dd3adbae8c145d680403ea1af18

* [TravelingSalesmanExact.jl]
https://github.com/ericphanson/TravelingSalesmanExact.jl
