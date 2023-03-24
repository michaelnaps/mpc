The work conducted here was initially completed in the Ohio State University undergraduate program with the intent of graduating with research distinction in the department of Mechanical Engineering. The requirements for said distinction was: 1). participation in an undergraduate research forum, 2). completion of research-related coursework and 3). the completion of a comprehensive thesis (published and linked below).

**Thesis:** https://kb.osu.edu/handle/1811/101702


___

## **Introduction**

The model predictive control (MPC) architecture is a powerful tool in the realm of control theory. It defines the process of looking into future states of a system (as governed by predetermined model equations) and selecting the most appropriate set of controls with respect to some cost/objective function. The set of commands and libraries presented here serves as my personal testing ground for the solutions to the MPC formulation - and is in no way a competitor with more professional libraries (see final note below).

The library is currently being optimized for use in **MicroPython** and thus has been modified so that it does not require a **numpy** dependency - removing the need to install third-party libraries on the microcontroller of interest. The **nno** control strategy, as referenced in my undergraduate thesis, can be found on the branch of the library titled **nno_save**. It will be reimplemented in the future. The following sections of the **readme** will discuss the MPC policy structure and show demonstrations when applicable.

**Final Note:** The class and associated functions shown here are in no way complete. It is my hope to use the model predictive control architecture as a platform for introducing myself to varying optimization strategies. More specifically, I intend to develop the library whenever I have freetime and new ideas as inspired by my studies.


___

## **Model Predictive Control**

### 1. Definition of a Model and Prediction Horizon
First, the generalized MPC will be defined so that each of the following sections can be spent investigating a method of finding the solution to the problem.

Let us first define our model in terms of a discrete linear/nonlinear dynamical system.

$$
    x_{k+1} = f(x_k, u_k)
$$

Where $x \in \mathbb{M} \subset \mathbb{R}^n$ defines the system's state, $u \in \mathbb{U} \subset \mathbb{R}^m$ defines the control injection and the subscript $k$ defines the $k$-th step in a discrete time series. We thus say that $f : \mathbb{M} \rightarrow \mathbb{M}$, or that it defines the translation between states in $\mathbb{M}$.

We can use this model to characterize a prediction horizon as a series of simulated steps starting at some initial position, $x_0$, and of some predetermined length, $P$. Using this idea, we can write the collection of state terms which make up the prediction horizon using set theory notation...

$$
    X = \lbrace x_0, \dots, x_{P+1}\ |\ x_0 \in \mathbb{M} \wedge x_{k+1} = f(x_k,u_k)\ \forall k \in \mathbb{N}_P \rbrace
$$

Where $\mathbb{N}_P$ denotes the set of whole numbers less than, but not equal to, $P$. In other words, $X$ consists of all of the states in the prediction horizon which obey the model, $f$, and start from a given initial position, $x_0$. We can similarly define the set of inputs as the list of controls which move the state forward. That is...

$$
    U = \lbrace u_0, \dots, u_{P}, 0\ |\ u_k \in \mathbb{U}\ \forall k \in \mathbb{N}_{P} \rbrace
$$

A placeholder $0$ is used to represent the control at $k=N_P+1$. It is important to note that in some notations $u_k$ is defined as $u_k \in \mathbb{U}(x_k)$; implying that the input is a member of the set of *allowable* controls at a given state, $x_k$. In the applications shown here, the set allowable controls are equivalent at all states - even if some may be considered impossible in the real-world. Particulary, $\mathbb{U} \subseteq \mathbb{U} (x)\ \forall x$.


### 2. Definition of an Objective Function and MPC

With the sets $X$ and $U$ defined we can characterize an objective function, $G$, as a collection of costs performed over the prediction horizon.

$$
    G(X,U) = \lbrace g(x,u)\ |\ \forall x \in X, \forall u \in U \rbrace
$$

Where the function, $g$, determines the instantaneous cost of a state and its corresponding control injection. The dominant goal in MPC is then to minimize the collective cost contained in $G$.

<!-- $$
\begin{aligned}
    \min_{u \in U} && G(X,U)
\end{aligned}
$$ -->


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