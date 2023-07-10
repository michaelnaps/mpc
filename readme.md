The work conducted here was initially completed in the Ohio State University undergraduate program with the intent of graduating with research distinction in the department of Mechanical Engineering. The requirements for said distinction was: 1). participation in an undergraduate research forum, 2). completion of research-related coursework and 3). the completion of a comprehensive thesis (published and linked below).

**Thesis:** https://kb.osu.edu/handle/1811/101702


___

## **Introduction**

The model predictive control (MPC) architecture is a powerful tool in the realm of control theory. It defines the process of looking into future states of a system (as governed by predetermined model equations) and selecting the most appropriate set of controls with respect to some cost/objective function. The set of commands and libraries presented here serves as my personal testing ground for the solutions to the MPC formulation - and is in no way a competitor with more professional libraries (see final note below).

I am currently in the process of splitting the class **ModelPredictiveControl()** into a system of subclasses. Each subclass will be dedicated to one portion of the standard MPC framework, and culminate into the MPC optimizer. These subclasses consist of the **Model()** class, the **Cost()** class and the **Optimizer()** class (which contains the optimization members). My hope is that this will present the library in a more readable/approachable form as well as make future implementations of MPC more robust.

<!--
The library is currently being optimized for use in **MicroPython** and thus has been modified so that it does not require a **numpy** dependency - removing the need to install third-party libraries on the microcontroller of interest. The **nno** control strategy, as referenced in my undergraduate thesis, can be found on the branch of the library titled **nno_save**. It will be reimplemented in the future. The following sections of the **readme** will discuss the MPC policy structure and show demonstrations when applicable.
-->

**Final Note:** The class and associated functions shown here are in no way complete. It is my hope to use the model predictive control architecture as a platform for introducing myself to varying optimization strategies. More specifically, I intend to develop the library whenever I have freetime and new ideas as inspired by my studies. Recently, I have identified that the library's biggest flaw is the consolidation of the entire process into a single MPC class. In the future, I hope to break the MPC class into sub-classes, helper functions, etc. for better generalization to other problems.


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

Where $U^\*$ represents the set of optimal policies as defined by the cost function solved for over the prediction horizon, and $X^\*$ is the optimal state predictions as accumulated through $U^\*$.


___

## **Gradient Descent**

**description coming soon**


___

## **Nonlinear Newton's Optimization**

**description coming soon**


___

## **Application:** Sphere World Environment

**description coming soon**

<p align='center'>
    <img src=./Examples/.figures/sphereworld.gif width=450>
</p>


___

## **Application:** Differential Drive Robot

<p align='center'>
    <img src=./Examples/.figures/donald.gif width=450>
</p>

___
**NOTE:** If a Hessian matrix is block diagonal then its inverse can be found by inverting each of its elements. In other words,

$$
    \texttt{diag}(D_1, D_2, D_3)^{-1} = \texttt{diag}(D_1^{-1}, D_2^{-1}, D_3^{-1})
$$

May be able to exploit this to remove **numpy** dependency from **nno**.
