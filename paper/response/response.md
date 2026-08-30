Original Manuscript ID: Access-2026-36061
Original Article Title: "Observability-Aware Virtual Force Sensing and Self-Calibration for Quadrotors"
(revised title: "Identifiability-Aware Virtual Force Sensing and Self-Calibration for Quadrotors")

To: IEEE Access Editor
Re: Response to reviewers

Dear Editor,

Thank you for allowing a resubmission of our manuscript, with an opportunity to address the reviewers' comments.

We are uploading (a) our point-by-point response to the comments (below) (response to reviewers, under "Author's Response Files"), (b) an updated manuscript with the changes marked in blue (as "Highlighted PDF"), and (c) a clean updated manuscript without highlights ("Main Manuscript").

Main changes:
- Title and terminology: "observability" replaced by "identifiability" throughout (Reviewer 3).
- New Table 1: comparison with momentum observers, ESO/disturbance observers, dead-zone/covariance management, Fisher-optimal input design, dual control, and recursive inertial identification (Reviewers 1, 5).
- Cramér–Rao bound restated as Proposition 2 with proof; the exact Fisher information under the random-walk force model derived in Section III (Eqs. (13)–(15)); new Fig. 14 (Reviewers 1, 3, 5).
- Classification signal $\tilde\sigma$ and information content $\sigma/r_s$ separated explicitly; new Table 6 (Reviewer 1).
- New Eq. (16) relating the threshold to the Cramér–Rao bound; new Section VII-I and Table 7 with the sensitivity study (threshold, dwell, window, process noise, accelerometer noise, maneuver intensity) (Reviewers 1, 5).
- New Section IV-B on the validity of the linearization (Reviewer 2).
- Clarifications for Reviewers 4 and 5; Figs. 8 and 9 re-run under identical conditions with a larger probe budget (Reviewer 3).

Best regards,
José Varela-Aldás et al.

======================================================================
Reviewer #1
======================================================================

Reviewer#1, Concern #1 (comparison with covariance management, Fisher-information input design, active system identification, and dual control; add a comparison table):

Author response: Agreed. The difference is not that we gate or excite, but that one closed-form scalar (the Schur complement, i.e. the marginal Fisher information on the mass) is at once the online test of whether the mass is separable from the vertical force, the gate signal, and the trigger and shaping criterion of the excitation. Momentum and ESO/disturbance observers assume the mass known and have no gate; dead-zone and covariance-management schemes gate on a generic excitation proxy that cannot tell hover from a maneuver (our energy-gated baseline shows this); Fisher-optimal input design and active identification plan the excitation offline; dual control accounts for information implicitly in the cost.

Author action: New Table 1 (what each family measures, its gate signal, whether it isolates the mass–vertical-force coupling, whether it restores identifiability) and a paragraph in the Introduction with the references [Han 2009; Chen et al. 2016; Ioannou and Sun 1996; Narendra and Annaswamy 1987; Mehra 1974; Morelli 2022; Mesbah 2018; Wüest et al. 2019; Burri et al. 2018; Boyacioglu et al. 2023].

----------------------------------------------------------------------
Reviewer#1, Concern #2 (Eqs. (8)–(12) treat d as constant, while the EKF models it as a random walk; extend the derivation or quantify the approximation error against window length, disturbance bandwidth, and $q_d$):

Author response: Correct. Under the random-walk model the exact marginal information on the inverse mass is a weighted Schur complement, $\sigma_{\mathrm{eff}}/r_s = t^{\top}\Sigma^{-1}t - t^{\top}\Sigma^{-1}M\,(M^{\top}\Sigma^{-1}M)^{-1}M^{\top}\Sigma^{-1}t$ with $\Sigma = r_s I + q_d\, C \otimes I_3$, $[C]_{kl} = \min(k,l) - 1$, which reduces to $\sigma/r_s$ for $q_d \to 0$. Evaluated on logged flight data (new Fig. 14(b)): for the implemented $q_d = 10^{-2}$ the information in a 1-s window is 12% of the constant-force value, and the ratio falls with the window length (0.25–4 s) and with $q_d$ (0–0.1). Consequences: (i) Proposition 2 is an optimistic bound for the implemented filter; scaled by this ratio it is of the scale of the empirical across-run spread of the mass under excitation (Fig. 14(a)). (ii) for the disturbance profiles (steps, sinusoid) and the $q_d$ range evaluated, the gate decision is unaffected: it thresholds the dimensionless $\tilde\sigma$ with margin, and at hover $\sigma_{\mathrm{eff}}$ stays at the noise floor for every $q_d$ tested. The random-walk variance is the bandwidth parameter of the implemented model; we did not sweep the physical disturbance frequency separately, and the manuscript now says so.

Author action: Derivation in Section III (Eqs. (13)–(15)), new Fig. 14, new paragraph after Proposition 2 in Section III, and the bound-versus-spread comparison in Section VII-I.

----------------------------------------------------------------------
Reviewer#1, Concern #3 ($\sigma$ is the dispersion of the thrust vectors, not of the direction; the normalization removes information relevant to the Cramér–Rao bound; separate the classification signal from the information content):

Author response: Correct on both counts. $\sigma$ becomes positive through changes in thrust magnitude, direction, or both; the "directional" wording was imprecise. Normalizing by $\sum_k T_k^2$ removes the thrust scale on purpose (one threshold across maneuvers) but discards information the bound retains: equal $\tilde\sigma$ with different thrust or noise levels means different Fisher information.

Author action: Definition of $\sigma$ rewritten after Eq. (10); text after Eq. (11) states what the normalization discards; $\tilde\sigma$ is now called the classification signal and $\sigma/r_s$ the information content throughout; new Table 6 lists both, plus the one-second Cramér–Rao mass uncertainty, for hover, five speeds, and the probe.

----------------------------------------------------------------------
Reviewer#1, Concern #4 (one normalized threshold is shown to work only in the reported environment; provide a sensitivity study over window, dwell, process noise, measurement noise, and maneuver intensity; relate the threshold to an allowable mass variance through the Cramér–Rao bound):

Author response: Both done. Threshold vs. bound: near hover $\sum_k T_k^2 \approx N(mg)^2$, so for a window that just opens the gate the Cramér–Rao bound reads $\mathrm{std}(\hat m)/m \gtrsim (1/g)\sqrt{r_s/(\tilde\sigma_{\min} N)}$ (new Eq. (16)): 7.6% for one second of data and 2.4% after ten with our parameters. The bound is a floor, not a guarantee: for a one-second window at or below the threshold, even an efficient unbiased estimator could not attain a standard deviation smaller than 7.6%, which is of the scale of the 5–7% hover bias the gate guards against. Under the same ideal constant-force assumptions, the floor crosses the observed 3.3% residual after approximately 5.3 seconds of accumulated admissible data and reaches 2.4% after ten seconds. These figures characterize only the best-case variance permitted by the information; they neither predict nor upper-bound the realized estimation error. Sensitivity (new Table 7; windy maneuver-to-hover transition of Fig. 7, one parameter at a time, five seeds each, RMS mass error over the hover; reference 2.2% gated vs 5.1% plain):
- Threshold: below 0.009 the gate re-opens on hover excursions (3.4–3.8%); at 0.015 it is erratic (5.1 ± 2.8%); at 0.03–0.06 it holds the mass to 0.4–0.6%. The threshold is limited not by the hover but by what must re-open the gate: with the 0.8-m budget the probe reaches $\tilde\sigma \approx 0.048$, so a threshold of 0.03 would be admissible for the probe and would hold the mass below 1%, but it would exclude the 3.8 m/s flight ($\tilde\sigma = 0.014$) from identification; we therefore retain 0.009 and accept the residual re-openings in windy hover.
- Dwell 0.1–1 s: gate never worse than plain, best at 0.25 s; long dwells make re-openings rare but larger.
- Window: 0.5 s too short (8.8%); 1–2 s hold at 2.2–3.1%.
- Process noise: gate helps for $q_m \le 10^{-5}$, counterproductive at $10^{-4}$; essential at $q_d = 10^{-3}$ (plain 53%, gated 6.9%), unnecessary at $q_d = 10^{-1}$ where the force absorbs everything (plain 1.8%). A loose force model hides the ambiguity instead of resolving it and is correct only if the mass never changes.
- Accelerometer noise 0–0.7 m/s$^2$: gated error stays at 2.2–2.5%; the plain drift itself shrinks, so the margin narrows.
- Maneuver intensity: unchanged at 5.9–8.4 m/s; at 3 m/s peak $\tilde\sigma$ never exceeds the threshold, neither filter identifies the mass, and both are vulnerable to the hover-onset transient (44% plain, 40 ± 44% gated). This is the regime the active probe exists for.
Transfer to other vehicles and IMU grades is not established by one simulated platform; the paper now says so at the end of Section VII-I. Equation (16) provides a necessary best-case information condition, not a sufficient accuracy guarantee, for selecting a threshold from $N$, $r_s$, and the desired mass uncertainty.

Author action: New Eq. (16) and paragraph in Section III; new Section VII-I with Table 7 (sensitivity), the offline window analysis, and Table 6; operating region stated explicitly (window 1–2 s, dwell about 0.25 s, $q_m \le 10^{-5}$, $q_d \le 10^{-2}$, threshold between hover excursions and probe level); the battery scripts are included with the released code.

----------------------------------------------------------------------
Reviewer#1, item 5a (references to remove): none were indicated. We re-checked the list and removed none; we will act on any specific indication.

======================================================================
Reviewer #2
======================================================================

Reviewer#2, Concern #1 (validity of the EKF with respect to the level of nonlinearity; scenarios such as rapid motion or rotated frames where it could be less applicable):

Author response: The model is linear in position, velocity, and force; the attitude enters as a known input, not as a state; drag is not estimated. The only nonlinear dependence on the state is $T/m$, whose linearization about $\hat m$ has a relative curvature error $(\delta m/m)^2$: below 1% for a 10% mass error, 18% for the 0.60 kg prior of Fig. 5, which still converges. The matched moving-horizon estimator (full nonlinear window, no local linearization) did not improve on the EKF, so the linearization is not the limiting factor. The scenarios named by the reviewer matter for other reasons: under rapid rotation a latency $\delta t$ between IMU and attitude gives an error of order $g\,\omega\,\delta t$ (0.5 m/s$^2$ at 5 rad/s and 10 ms), a synchronization requirement; and since the force is in the world frame, a body-fixed disturbance (drag at speed) must be followed by the random walk as the attitude turns it, the bandwidth limit already noted in Section II. A body-frame force state or an unscented/iterated update would be the drop-in changes if either dominated.

Author action: New Section IV-B, "Validity of the linearization".

======================================================================
Reviewer #3
======================================================================

Reviewer#3, Key point (the paper addresses identifiability, not observability; replace the term, including in the title; see Boyacioglu et al. 2023 for observability of mass in this kind of system):

Author response: Agreed. The quantity analyzed is the identifiability of the parameters $(m, d)$ in the accelerometer regressor; observability of the augmented state coincides with it only because position and velocity are measured directly.

Author action: Title changed to "Identifiability-Aware Virtual Force Sensing and Self-Calibration for Quadrotors"; "observability"/"observable" replaced by "identifiability"/"identifiable" throughout (abstract, keywords, section titles, Table 2, figure captions and figure legends, conclusion); "observability" kept only for the nonlinear observability of the augmented state in Section III, where a sentence now states the distinction and cites [Boyacioglu et al. 2023].

----------------------------------------------------------------------
Reviewer#3, Minor points:

Author response and action:
- "Mass is not a force" (p. 1): reworded in the abstract, the Introduction, and Section II: a mass error, which enters the reading through the thrust-to-weight ratio, is what is indistinguishable from a vertical force; Section II now says "the weight of an added mass".
- Eq. (12) as a Proposition: done; it is now Proposition 2 (Cramér–Rao bound on the mass) with its proof.
- Smoothing (p. 6): a paragraph in Section V explains it. The filter's $q_d$ is set for the bandwidth of the force reading, so its output keeps sample-to-sample noise at 100 Hz that the controller would pass into the thrust command; the moving average is a low-pass on the actuation path, tunable independently of the estimation bandwidth (0.12 s for the force; 2 s for the mass, which sets the hover equilibrium of the prediction model).
- Optimal excitation for identifiability in flight systems (Section V-B): [Morelli 2022] is now cited there, together with [Mehra 1974], as the input-design problem we do not solve in continuous form.
- Fig. 8: regenerated from 50-s runs (previously 35 s), showing 34 s of hover after the probe is withdrawn with the mass held at the identified value. In doing so we found that, in the submitted version, the passive runs of Fig. 8 had been flown without wind and the active runs with it, and that the 0.4-m probe budget left the probe's $\tilde\sigma$ at the gate threshold. Both experiments (Figs. 8 and 9) were therefore re-run under identical wind-free conditions with the budget raised to 0.8 m and an explicit, empirically chosen withdrawal rule (15 s of gate-open time; the constant-force bound of Eq. (16) for the information accumulated in that time is about 1%, the order of the residual observed). Results: recovery from the 0.70-kg prior to $0.4 \pm 0.2$% (N = 5) with the most informative probe; all shapes at this budget exceed the threshold and the achieved $\tilde\sigma$ (0.033 to 0.048) sets the settling time (14.3 to 8.3 s, $p = 0.004$), which is now what Fig. 9 reports. Sections V-B, VII-D, VII-E, VII-G and Tables 3, 6 were updated accordingly.

----------------------------------------------------------------------
Reviewer#3, Typographical and formatting notes:

Author action: "Fig." at the start of a sentence changed to "Figure" (11 instances); "Section VIII concludes the paper."; "body attitude rate (body rate for short)" at the definition of $\omega$; comma after "separated" removed; comma after "mass" added; "newtons squared" kept in lowercase, as SI prescribes for unit names spelled out (the italic $N$ is reserved for the window length, so we did not switch to the symbol either); we hope the reviewer accepts this; "Dividing out" changed to "Normalizing by"; Table 2 now appears on the page of its first mention, and Table 3 was moved next to its first mention as well; Fig. 4 regenerated at double-column width and Fig. 13 as a single-column stack, both with fonts at caption size; the per-panel headers of Fig. 4 were reduced to the axis labels (x), (y), (z), with the fitted slopes given in the text.

======================================================================
Reviewer #5
======================================================================

Reviewer#5, Concern #1 (Contribution 1 reads like an external-disturbance estimation method; what is the fundamental difference from an ESO or an unknown-disturbance observer?):

Author response: The force reading itself is not different: it is the accelerometer residual, the same signal an ESO or a disturbance observer builds its disturbance state on. Those observers assume the model, and hence the mass that scales the residual, known; they have no notion of whether the mass is currently separable from the vertical force, and no way to act when it is not. Our contribution is the calibration layer around the residual: (i) an online, closed-form test of that separability (the Schur complement, i.e. the marginal Fisher information on the mass), (ii) a gate that freezes the mass when the test fails, and (iii) an excitation triggered and shaped by the same test.

Author action: New Table 1 and the accompanying paragraph in the Introduction, which now states explicitly that the disturbance state of an ESO/UDO is the same residual our reading is built on and that what we add is the calibration layer.

----------------------------------------------------------------------
Reviewer#5, Concern #2 (how is unidentifiability quantitatively defined; what criterion decides it?):

Author response: The criterion is the normalized Schur complement of Eq. (11), computed over a 1-s sliding window from the known thrust and attitude, falling below a threshold: $\tilde\sigma < \tilde\sigma_{\min} = 0.009$ (Eq. (18)). Eq. (11) is exactly zero in stationary hover (Proposition 1) and grows with the dispersion of the thrust vector; Table 6 reports its value per regime and Section VII-I its sensitivity.

Author action: The contribution bullet and Section IV-C now state the criterion explicitly ("the mass is declared unidentifiable, quantitatively, when $\tilde\sigma$ falls below $\tilde\sigma_{\min}$"). Equation (16) states the corresponding Cramér–Rao lower bound as a necessary best-case information condition, not an accuracy guarantee.

----------------------------------------------------------------------
Reviewer#5, Concern #3 (the model of Eq. (1) contains several physical parameters; how is the accuracy of their calibration guaranteed?):

Author response: Few of them enter the reading. $g$ is known; the drag coefficient is not identified but set to zero, so any drag is lumped into the force and read as part of it; $\tau_{rc}$ enters only the controller's prediction model; the mass is estimated. The single calibration the reading depends on is the thrust map that gives $T$; its error is characterized in Section VII-H: a 10% thrust mismatch couples almost one-to-one into the mass estimate and barely into the force.

Author action: A paragraph at the end of Section II-B now lists, parameter by parameter, what is assumed, lumped, estimated, or calibrated, with the pointer to Section VII-H.

----------------------------------------------------------------------
Reviewer#5, Concern #4 (Eq. (12) is said to give a boundedness property of the inverse mass, emphasized as a contribution, but it is not experimentally validated):

Author response: Eq. (12) is not a boundedness property of $\beta$; it is the Cramér–Rao lower bound on the variance of any unbiased estimate of $\beta$, i.e. the statement that the Schur complement is the marginal Fisher information on the mass. We agree that it deserved a clearer statement and an experimental check.

Author action: Eq. (12) is now Proposition 2 with its proof; new Section VII-I and Fig. 14(a) compare the bound with the empirical across-run spread of the mass estimate at six flight speeds (the bound is of the scale of the spread under excitation once the random-walk force model is accounted for, Section III), and Table 6 reports the bound per regime.

----------------------------------------------------------------------
Reviewer#5, Concern #5 (what does "probe" refer to; a physical sensor?):

Author response: Not a sensor. The probe is a small, bounded excursion added to the position reference of the controller (Eq. (22): a lateral circle and/or a vertical bob of amplitude within a budget $\rho$), injected only while the mass is needed and unidentifiable, and withdrawn afterwards.

Author action: Defined at its first use in the Introduction ("a small bounded excursion of the position reference (not a sensor)") and in Section V-B.

----------------------------------------------------------------------
Reviewer#5, Concern #6 (does the objective of Eq. (16), now Eq. (20), need normalization, since terms of different magnitude may dominate?):

Author response: The weights are the normalization. With the values of Table 3, a position error of 0.1 m, a velocity error of 0.25 m/s, an attitude error of 0.3 rad and a thrust deviation of 2.5 N each contribute about 1.8 to the stage cost, so no term dominates at the errors the controller operates at.

Author action: Sentence added after Eq. (20) stating this.

----------------------------------------------------------------------
Reviewer#5, Concern #7 (the optimization objective is given but not the final controller expression):

Author response: The controller is the receding-horizon law: at each step the problem is solved from the current state, mass, and force estimates, the first input of the optimal sequence is applied, and the horizon shifts.

Author action: New Eq. (21), $u_k = u_0^{\star}(\hat x_k, \hat m, \hat d_k)$, with the sentence describing the receding-horizon rule, in Section V-A.

----------------------------------------------------------------------
Reviewer#5, Concern #8 (with all parameters known, would the difference between the energy delivered and the kinetic energy received not directly give the disturbance; what is the conceptual difference from virtual force sensing?):

Author response: An energy balance can indeed be formed, but it is a scalar (it loses the direction of the force), it is an integral (the force enters as $\int d \cdot v\,dt$, so it lags), and it vanishes wherever the velocity does, which is precisely the hover where the force matters most. The specific-force reading is a vector, instantaneous, and independent of the velocity.

Author action: Paragraph added in Section III after the discussion of the algebraic measurement.

----------------------------------------------------------------------
Reviewer#5, item 5a (references to remove): none were indicated; we removed none and will act on any specific indication.

======================================================================
Reviewer #4
======================================================================

Reviewer#4, Concern #1 (report the compute platform for the "<1 ms per step at 100 Hz" claim, and say so if it is not flight-representative):

Author response: The timings were measured on a laptop-class processor (Intel Core Ultra 9 275HX, one core per solver), not on an embedded flight computer; the method has not been validated on embedded hardware, and we do not claim it.

Author action: Platform stated in Section V-A where the solve time is first given, and referenced again in Section VII-F.

----------------------------------------------------------------------
Reviewer#4, Concern #2 (why the injected-error ranges of Section VII-H, 0.4 m/s$^2$ bias and 10% scale/thrust, were chosen):

Author response: They bracket what a real platform presents before any in-field calibration: the bias range covers the uncalibrated initial offset of consumer MEMS accelerometers (tens of mg, a few tenths of m/s$^2$), the scale-factor range covers their sensitivity tolerance of a few percent with a wide margin, and the thrust range covers the change of the thrust map over a flight from battery-voltage sag and propeller wear, of the order of 10%.

Author action: Sentence added in Section VII-H.

----------------------------------------------------------------------
Reviewer#4, Concern #3 (clarify the degradation of the $d_z$ correlation with speed in Table 3 (now Table 4)):

Author response: The ground-truth channel contains only the applied wind, whereas the reading also contains the lumped aerodynamic drag ($c = 0$ in the estimator). The figure-eight has a vertical velocity component, so the lumped drag acquires a vertical part that grows with speed (its driver $v_z\|v\|$ rises from 0.02 to 14 m$^2$/s$^2$ RMS across the battery); it is uncorrelated with the applied steps, and the vertical steps are half the size of the horizontal ones (0.8 N against 1.5 N on $x$), so the same uncorrelated content costs more correlation on that axis. The absolute vertical error moves the other way, from 0.62 N in hover, where the mass–force ambiguity biases the plain filter's $d_z$ (a bias the correlation does not see), to 0.29 N at 10.2 m/s; the dip to 0.91 at 6.1 m/s is comparable to the run-to-run spread ($\pm 0.02$).

Author action: Paragraph added in Section VII-A, and Table 4 moved next to it.
