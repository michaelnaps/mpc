The work conducted here was initially completed in the Ohio State University undergraduate program with the intent of graduating with research distinction in the department of Mechanical Engineering. The requirements for said distinction was: 1). participation in an undergraduate research forum, 2). completion of research-related coursework and 3). the completion of a comprehensive thesis (published and linked below).

**Thesis:** https://kb.osu.edu/handle/1811/101702

___

## **Introduction**

The model predictive control (MPC) architecture is a powerful tool in the realm of control theory. It defines the process of looking into future states of a system (as governed by predetermined model equations) and selecting the most appropriate set of controls with respect to some cost/objective function. The set of commands and libraries presented here serves as my personal testing ground for the solutions to the MPC formulation - and is in no way a competitor with more professional libraries.

For some animations of the system at work, you can skip the description and scroll to the bottom of the *readme.md*.

**Final Note:** The classes and associated functions shown here are in no way complete. It is my hope to use the model predictive control architecture as a platform for introducing myself to various optimization strategies. I intend to develop the library whenever I have free time and new ideas as inspired by my studies. I have identified that the library's biggest flaw is the lack of alternatives to gradient descent-based approaches, and the inexistance of any LP-oriented optimization strategies. These will hopefully be explored more thoroughly in the future.

___

## **Model Predictive Control**

Overall, the MPC decision scheme follows the diagram below.

<p align='center'>
    <img src=./Examples/.figures/mpc_decision_structure.png width=450>
</p>

### 1. *Definition of a Model and Prediction Horizon*
First, the generalized MPC will be defined so that each of the following sections can be spent investigating a method of finding the solution to the problem.

Let us first define our model in terms of a discrete linear/nonlinear dynamical system.

$$
    x_{k+1} = F(x_k, u_k)
$$

Where $x \in \mathbb{M} \subset \mathbb{R}^n$ defines the system's state, $u \in \mathbb{U} \subset \mathbb{R}^m$ defines the control injection and the subscript $k$ defines the $k$-th step in a discrete time series. We thus say that $F : \mathbb{M} \rightarrow \mathbb{M}$, or that it defines the translation between states in $\mathbb{M}$.

We can use this model to characterize a prediction horizon as a series of simulated steps starting at some initial position, $x_0$, and of some predetermined length, $P$. Using this idea, we can write the collection of state terms which make up the prediction horizon using set theory notation...

$$
    X = \\{ x_0 \wedge x_{k+1} : x_{k+1} = F(x_k, u_k)\ \forall k < P : k \in \mathbb{Z} \\}
$$

Where $X$ consists of all of the states in the prediction horizon which obey the model, $F$, and start from a given initial position, $x_0$. We can similarly define the set of inputs as the list of controls which move the state forward.

$$
    U = \\{ u_k : u_k \in \mathbb{R}^m\ \forall k < P : k \in \mathbb{Z} \\}.
$$

A placeholder $0$ is used to represent the control at $k=N_P+1$. It is important to note that in some notations $u_k$ is defined as $u_k \in \mathbb{U}(x_k)$; implying that the input is a member of the set of *allowable* controls at a given state, $x_k$. In the applications shown here, the set allowable controls are equivalent at all states - even if some may be considered impossible in the real-world. Particulary, $\mathbb{U} \subseteq \mathbb{U} (x)\ \forall x$.

### 2. *Definition of Period and Terminal Costs*

Using the aforementationed sets, a cost function can be defined which assigns a 'score' to each of the *period* costs, or the state and its associated input, as well as the *terminal* cost, or the final state evaluated at the end of the prediction horizon.

$$
    \begin{aligned}
        g(x_k,u_k) = \text{period cost} \\
        g_P(x_P) = \text{terminal cost}
    \end{aligned}
$$

Where the functions $g_k, g_P \in \mathbb{R}$ are defined before the start of the controller, making the set of all prediction horizon costs

$$
    G = \\{ g_k(x_k, u_k) \wedge g_P(x_P) : \forall x_k \in X, u_k \in U \\}.
$$

Where $G$ is simply used here to represent the set of costs which exist for the sets defined by $X$ and $U$. From this, it is possible to define an optimization problem over the entire prediction horizon.

### 3. *Definition of the Optimization Problem*

The optimization now becomes a minimization of the total cost over the $P$-prediction horizons.

$$
    X^\*, U^\* = \min_{X,U} \left( \sum_{k=0}^{P-1} g_k(x_k, u_k) + g_P(x_P) \right)
$$

Where $U^\*$ represents the set of optimal policies as defined by the cost function solved for over the prediction horizon, and $X^\*$ is the optimal state predictions as accumulated through $U^\*$. It is important to note that the optimal states over the prediction are simply a *byproduct* of the optimization problem and are at most used as a tuning reference post-runtime.

In most, if not all cases, the optimization problem can be further simplified by the assumption that the intial state, $x_0$, does not change as the optimizer iterates. While the initial state impacts the magnitude of the total cost, it is a constant value as no control can change its position. For this reason it can be remove from the cost space completely and we can rewrite the cost space definition as

$$
    G = \\{ g_k(x_{k+1}, u_k)\ \forall x_{k+1} \in X, u_k \in U \\}
$$

and replacing the optimization statement with

$$
    X^\*, U^\* = \min_{X,U} \\ \sum_{k=0}^{P-1} g_k(x_{k+1}, u_k) ||.
$$

In this form we ignore the initial state, giving a slightly more concise cost function.

___

## **Iterative Minimization Algorithm**

Now that the optimization problem is defined more thoroughly, the steps used to solve it can be expressed. First, let us define the stopping point to the optimization process. This occurs when the cost function is at a *local minimum*, or when the partial derivative w.r.t. the inputs of the cost function is equal to 0.

$$
    \nabla G = \frac{\partial}{\partial U} \left( \sum_{k=0}^{P-1} g_k(x_{k+1}, u_k) \right) = 0 \in \mathbb{R}^{m \times P}
$$

In practice, it will be more realistic to attempt to find an *approximately* optimal solution. That is,

$$
    ||\nabla G|| \leq \varepsilon
$$

where $\varepsilon \in \mathbb{R}$ and represents the first-order optimality stopping condition. It has been proven (see references below) that the derivative of a continuously differentiable function always points towards the maximum of said function. For this reason, we define the iterative search algorithm as

$$
    U_{i+1} = U_i - \beta(X,U) \nabla G
$$

where we will select the step-size function $\beta(X,U)$ for various use cases.

___

## **Nonlinear Gradient Descent**

In the nonlinear gradient descent (NGD, or steep-descent) algorithm, the step-size function $\beta(X,U) = \alpha \in \mathbb{R}$. That is,

$$
    U_{i+1} = U_i - \alpha \nabla G.
$$

In this case, the optimization takes the simplest available step towards the minimum. This method is efficient in cases where the set $X$ (and subsequently $U$) are large. Other optimization approaches will prove to have lower efficiency in these cases.

___

## **Nonlinear Newton's Optimization**

The nonlinear Newton's optimization makes the assumption that the cost function approximates a quadratic function and utilizes the second derivative of $G$ (or, hessian) to derive the nearest optimal minimum. The hessian is defined as

$$
    \nabla^2 G = \frac{\partial}{\partial U}(\nabla G).
$$

We thus set the step-size function to

$$
    \beta(X,U) = \alpha (\nabla^2 G)^{-1}
$$

making the optimization

$$
    U_{i+1} = U_i - \alpha (\nabla^2 G)^{-1} \nabla G.
$$

This method can be proven to be exact with $\alpha=1$ and in cases where the objective function is strictly quadratic. In cases where the objective is not quadratic, it is common to take $\alpha < 1$ and iterate till below the threshold $\varepsilon$.

While this method often converges in many fewer steps than the NGD method, it is computationally much more expensive. Implying that in problems where $X$ is large, it may not be applicable.

___

## **Application:** Sphere World Environment

**description coming soon**

<p align='center'>
    <img src=./Examples/.figures/sphereworld.gif width=450>
</p>

___

## **Application:** Differential Drive Robot

<p align='center'>
    <img src=./Examples/.figures/donald.gif width=350>
    <img src='https://github.com/michaelnaps/roomba_draws/blob/master/.figures/drawhat_n50.gif' width=350>
</p>
