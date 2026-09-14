# Paper draft — Methods and Experiments

Status markers: `[ ]` not started · `[~]` drafted, needs check · `[x]` settled.

Workflow: see `/Users/declanbohley/.claude/plans/experiments-are-now-done-compressed-clover.md`
for the process this file is built under. Every quantitative claim below must trace to a
specific session doc (`docs/sessions/NNN-*.md`) or a rerunnable command — no invented or
rounded-for-flow numbers. Open items from `CLAUDE.md`'s "Open questions" are stated as
limitations, not settled fact, wherever they touch a section.

## Methods

### M1. Delay characterization / forecast horizon `[~]`

\subsubsection{Delay characterization}
Respiratory-gated needle insertion requires the controller to act on the target's position at the instant insertion actually completes, not at the instant it is measured. Because the target is continuously in motion, any lag between measurement and action introduces positional error proportional to the target's velocity during that lag. Therefore a forecast horizon should be defined that the system can see in order to account for these delays, consisting of all of the delay sources.
\begin{equation}
h = \tau_s + \tau_c + \tau_{cl}(\omega_r) + T_{ins}
\label{eq:horizon}
\end{equation}

Where $\tau_s$ is lag from the sensor to the targets true position. This includes sensor latency but also the contact mechanics settling. $\tau_c$ is computation delay for the estimation and prediction latency. $\tau_{cl}(\omega_r)$ is the tracking lag of the needle-position servo, which is defined by how the lead compensated closed-loop controller is characterized. $T_{ins}$ is the flight time of the needle to its designated position.

> **Resolved**: the time-advance paragraph (settled decision 4) is Eq.~\ref{eq:prediction}
> in M2 — prediction is $\theta \to \theta + \omega_r h$, so harmonic $k$ rotates by
> $k\omega_r h$, never a single common-phase shift added to every harmonic.
> **Pending**: full derivation of $\tau_{cl}(\omega_r)$ and the open unity-feedback-vs-
> actual-architecture caveat (session 019) still belong in M3, cross-referenced from here —
> not yet written (M3 currently only sets up the closed loop $\tau_{cl}$ is computed from).

### M2. EKF signal construction `[~]`

\subsubsection{Signal construction}

The respiratory model as well as the collected data characterize the breath cycle as periodic with asymmetric inhale and exhale patterns. Real human breathing causes small shifts in the frequency of the breathing.\textcolor{red}{reference collected breathing data} What we exploit here is only the property common to any adequate description of that motion: periodicity itself. Any sufficiently regular periodic signal admits a Fourier decomposition regardless of the mechanism generating it, so the signal is modeled as a finite harmonic sum with a slowly-varying phase:

\begin{equation}
x(t) = a_0 + \sum_{k=1}^{K} A_k \sin\!\big(k\,\theta(t) + \phi_k\big), \qquad \dot\theta(t) = \omega_r(t)
\label{eq:harmonic_model}
\end{equation}

where $\omega_r(t)$ drifts overtime as it relates to the breath to breath variability of a given patient. In order to construct the signal for online tracking a calibration window must be recorded. The longer this window is the more accurate the initial estimate may be but due to the unrealistic nature of having a patient wait extended periods of time a window that captures 15-30 breath cycles is used. From this a Fast Fourier transform is taken to get an initial estimate $\hat{\omega_r}$. The frequency resolution of an N-sample window is fixed by its duration:
\begin{equation}
\Delta f = \frac{1}{N T_s}
\label{eq:freq_resolution}
\end{equation}

Now given the frequency estimate $\sin(k\hat\omega_r t)$ and $\cos(k\hat\omega_r t)$  are now known for harmonic regression.


\begin{equation}
x(t) \approx a_0 + \sum_{k=1}^{K_{max}} \Big[\alpha_k \sin(k\hat\omega_r t) + \beta_k \cos(k\hat\omega_r t)\Big]
\label{eq:harmonic_regression}
\end{equation}

Writing this in matrix form $y = X\beta + \varepsilon$, where $\beta = [a_0, \alpha_1, \beta_1, \ldots, \alpha_{K_{max}}, \beta_{K_{max}}]^T$. 

\begin{equation}
\begin{aligned}
X_{n,1} &= 1 \\
X_{n,2k} &= \sin(k\hat\omega_r t_n) \\
X_{n,2k+1} &= \cos(k\hat\omega_r t_n), \quad k=1,\ldots,K_{max}
\end{aligned}
\label{eq:design_matrix}
\end{equation}

Ordinary least squares then is used.

\begin{align}
\hat\beta &= \left(X^TX\right)^{-1}X^Ty \label{eq:ols_estimate}\\
\mathrm{Cov} &= \sigma^2 \left(X^TX\right)^{-1} \label{eq:ols_cov}
\end{align}

Amplitude and phase per harmonic follow as


\begin{equation}
A_k = \sqrt{\alpha_k^2 + \beta_k^2}, \qquad \phi_k = \operatorname{atan2}(\beta_k, \alpha_k)
\label{eq:amp_phase}
\end{equation}

K is then chosen to recover 95\% of total energy.

Batch identification assumes stationarity over the calibration window; real respiratory rate and depth drift breath to breath, motivating an online, recursive estimator. For a discrete-time nonlinear system with state $\boldsymbol{s}$, process model $f$, and measurement model $h$. An extended Kalman filter is used.

\begin{align}
\hat{\mathbf{s}}_{k|k-1} &= f\!\left(\hat{\mathbf{s}}_{k-1|k-1}\right) \label{eq:ekf_predict_state}\\
P_{k|k-1} &= F_k\, P_{k-1|k-1}\, F_k^T + Q \label{eq:ekf_predict_cov}
\end{align}

\begin{align}
\nu_k &= z_k - h\!\left(\hat{\mathbf{s}}_{k|k-1}\right) \label{eq:ekf_innovation}\\
S_k &= H_k\, P_{k|k-1}\, H_k^T + R \label{eq:ekf_innov_cov}\\
\mathbf{K}_k &= P_{k|k-1}\, H_k^T\, S_k^{-1} \label{eq:ekf_gain}\\
\hat{\mathbf{s}}_{k|k} &= \hat{\mathbf{s}}_{k|k-1} + \mathbf{K}_k \nu_k \label{eq:ekf_update_state}\\
P_{k|k} &= \left(I - \mathbf{K}_k H_k\right) P_{k|k-1} \left(I - \mathbf{K}_k H_k\right)^T + \mathbf{K}_k R \mathbf{K}_k^T \label{eq:ekf_update_cov}
\end{align}

\begin{figure*}[t]
    \centering
    \includegraphics[width=.99\linewidth]{Images/RealStateDiagram.png}
    \caption{The gated needle insertion process involves four steps: (1) The robot base and wrist bring the needle to a position suitable for insertion. (2) The contact sensor is brought within range of the torso to maintain contact and sense breathing. (3) Once the breathing motion is detected, the needle is driven into the torso. (4) The needle is allowed to passively move with the breathing motion, applying a gated insertion when the suitable breathing stage is detected.}
    \label{fig: state_mach}
\end{figure*}

Where $F_k = \partial f/\partial \mathbf{s}\big|_{\hat{\mathbf{s}}_{k-1|k-1}}$ and $H_k = \partial h/\partial \mathbf{s}\big|_{\hat{\mathbf{s}}_{k|k-1}}$ are the Jacobians linearizing $f$ and $h$ about the current estimate. $Q$ is the process-noise covariance, $R$ the measurement-noise covariance, $P$ the state-error covariance, $S_k$ the innovation covariance, and $\mathbf{K}_k$ the Kalman gain. In our case, the sensor is scalar, so $H_k$ is a $1\times n$ row vector, $R$ and $S_k$ are scalars, and Eq.~\ref{eq:ekf_gain} reduces to a division rather than a matrix inversion.

With harmonic order $K$ being fixed. the state has dimension $n = 2K + 3$.

\begin{equation}
\mathbf{s} = \begin{bmatrix} a_0 & A_1 & \phi_1 & \cdots & A_K & \phi_K & \theta & \omega_r \end{bmatrix}^T
\label{eq:state_vector}
\end{equation}

The absence of a good model to know how a patients breath rate might drift this is absorbed into $Q$. 

\begin{align}
\theta_k &= \theta_{k-1} + \omega_{r,k-1}\, T_s \label{eq:process_theta}\\
s_k(i) &= s_{k-1}(i), \qquad i \neq \theta \label{eq:process_rw}
\end{align}

so $F_k$ (Eq.~\ref{eq:ekf_predict_cov}) is the identity matrix except for a single $T_s$ entry coupling $\omega_r$ into the $\theta$ row.

The scalar measurement function is the harmonic sum evaluated at the current phase estimate:

\begin{equation}
h(\mathbf{s}) = a_0 + \sum_{k=1}^{K} A_k \sin(k\theta + \phi_k)
\label{eq:meas_model}
\end{equation}

The row-vector Jacobian $H_k$ is analytic:
\begin{align}
\frac{\partial h}{\partial a_0} &= 1 \label{eq:jac_a0}\\
\frac{\partial h}{\partial A_k} &= \sin(k\theta + \phi_k) \label{eq:jac_Ak}\\
\frac{\partial h}{\partial \phi_k} &= A_k \cos(k\theta + \phi_k) \label{eq:jac_phik}\\
\frac{\partial h}{\partial \theta} &= \sum_{k=1}^{K} k\, A_k \cos(k\theta + \phi_k) \label{eq:jac_theta}\\
\frac{\partial h}{\partial \omega_r} &= 0 \label{eq:jac_omega}
\end{align}

With $\mathbf{s}$, $F_k$, $H_k$, $Q$, $R$, and $P$ all now defined in place above (Eqs.~\ref{eq:ekf_predict_cov}--\ref{eq:ekf_update_cov}), each is set from the calibration recording:

\begin{itemize}
\item \textbf{$\hat{\mathbf{s}}_0$} --- read directly from the batch regression ($a_0, A_k, \phi_k, \hat\omega_r$), with $\theta$ initialized at the fitted phase value at the end of the calibration window.
\item \textbf{$P_0$} --- the $a_0/A_k/\phi_k$ block follows from the regression's coefficient covariance, Eq.~\ref{eq:ols_cov}, converted to amplitude/phase form via a first-order (delta-method) Jacobian; $\omega_r$'s initial variance is set from the frequency search's resolution, $2\pi/(N T_s)$.
\item \textbf{$R$} --- estimated from sensor variance during a still/breath-hold segment where available
\item \textbf{$Q$} --- estimated empirically by refitting each breath in the calibration recording separately, computing the variance of each state across those per-breath fits, and rescaling from breath-timescale to sample-timescale:
\end{itemize}

\begin{equation}
Q_i \approx \mathrm{Var}_{\text{breath-to-breath}}(i) \cdot \frac{T_s}{T_{breath}}
\label{eq:Q_construction}
\end{equation}

A single scalar can then be applied across this diagonal to adjust its performance.

$Q$ is diagonal, so $Q_i$ above denotes its $i$-th diagonal entry. Two states in $Q$ are exceptions to Eq.~\ref{eq:Q_construction}. $\theta$ has no independent per-breath estimate of its own — it is the running phase, not a fitted quantity — so it inherits $\phi_1$'s variance directly. $\omega_r$ cannot be estimated the same way at all: the per-breath refits above hold $\hat\omega_r$ fixed by construction, so there is no breath-to-breath spread in it to measure. Its entry instead comes from re-estimating the frequency independently over several overlapping multi-breath windows spanning the calibration recording and taking the variance across those windows — a single breath is too short a window to resolve frequency precisely enough for its scatter to be meaningful. When the calibration recording is too short to form at least two such windows, this falls back to the same resolution-based value used to seed $\omega_r$ in $P_0$.

The filter's live estimate is advanced to the required horizon by advancing the phase state directly and re-evaluating the harmonic sum at that advanced phase:

\begin{align}
\theta &\rightarrow \theta + \omega_r h, \\ r(t) &= \hat{x}(t+h) = a_0 + \sum_{k=1}^{K} A_k \sin\!\big(k(\theta + \omega_r h) + \phi_k\big)
\label{eq:prediction}
\end{align}

> **Still open, not yet fixed in the text above**: (1) "15-30 breath cycles" for the
> calibration window is not verified against any config or session doc; (2) the coarse
> frequency estimate is described as FFT-only, but the code cross-checks against an
> autocorrelation estimate too (`coarse_omega`); (3) whether the state-machine figure
> belongs here or should move to M4 is still undecided.

### M3. Lead compensator design `[~]`

\subsubsection{Compensation and Needle Drive}

When the needle is given a position-control reference, the needle actuation axis is
identified as a second-order system from its velocity step response. Since position is the time-integral of velocity, the plant seen by the
position-tracking loop carries an additional integrator:
\begin{equation}
G(s) = \frac{K_p\,\omega_n^2}{s\,\left(s^2+2\zeta\omega_n s+\omega_n^2\right)}
\label{eq:plant}
\end{equation}
Lead compensation is applied to reduce the tracking lag of this loop:
\begin{equation}
C(s) = K_c\,\frac{s+z}{s+p}, \qquad p > z
\label{eq:lead_compensator}
\end{equation}
$z$, $p$, and $K_c$ are chosen by a constrained search rather than fixed a priori:
candidates are restricted to those achieving at least a target phase margin and a
gain-crossover frequency held safely below the plant's natural frequency $\omega_n$; among
the candidates that satisfy both, the one giving the lowest residual closed-loop tracking
lag $\tau_{cl}(\omega_r)$ at the nominal breathing rate is selected, subject to a further
constraint bounding how much the compensator's high-frequency gain may amplify real
position-measurement noise into spurious corrections.

\textcolor{red}{Pending: derive $\tau_{cl}(\omega_r) = -\angle T(j\omega_r)/\omega_r$ from this closed loop, and state the open unity-feedback-vs-actual-architecture caveat (session 019) — \emph{residual\_lag}/\emph{closed\_loop\_response} assume standard unity feedback $T=L/(1+L)$, but \emph{\_hold\_standoff()} actually commands reference + correction, an algebraically different closed loop.}

### M4. State machine `[ ]`

## Experiments

### E1. Needle-system identification for lead compensation `[ ]`

### E2. Lag / delay measurement `[ ]`

### E3. Approach experiments `[ ]`

### E4. EKF tuning experiments `[ ]`

### E5. Needle insertion experiments `[ ]`
