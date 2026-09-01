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

In what follows, each reviewer comment is quoted in italics, followed by our response and the action taken in the manuscript.

Best regards,
José Varela-Aldás et al.

======================================================================
Reviewer #1
======================================================================

Reviewer#1, General comment:

> This manuscript presents an observability-aware virtual force-sensing framework for quadrotors. The main idea is to estimate the external specific force from the accelerometer residual, identify the vehicle mass only when sufficient excitation is available, freeze the mass update during poorly observable flight, and introduce an active excitation maneuver when recalibration is required. The use of the Schur complement as a common signal for mass-identifiability assessment, estimator gating, and probe selection is technically interesting. The manuscript is generally well organized, and the software-in-the-loop results indicate meaningful improvements in mass-estimation consistency and disturbance rejection.

Author response: We thank the reviewer for the accurate summary and for the constructive technical comments, each addressed below.

----------------------------------------------------------------------
Reviewer#1, Comment 1:

> The manuscript acknowledges that parameter-update freezing, persistent-excitation tests, and active input design are individually classical. The claimed contribution is that the same Schur-complement quantity governs gating and active excitation. This is potentially useful, but the distinction from existing covariance-management methods, Fisher-information-based input design, active system identification, and dual-control approaches remains qualitative. The introduction should include a more systematic comparison explaining what existing methods measure, what signal they use for gating, whether they consider the mass–external-force ambiguity, and whether they actively restore observability. A comparison table would make the methodological contribution considerably clearer.

Author response: Agreed. The difference is not that we gate or excite, but that one closed-form scalar (the Schur complement, i.e. the marginal Fisher information on the mass) is at once the online test of whether the mass is separable from the vertical force, the gate signal, and the trigger and shaping criterion of the excitation. Momentum and ESO/disturbance observers assume the mass known and have no gate; dead-zone and covariance-management schemes gate on a generic excitation proxy that cannot tell hover from a maneuver (our energy-gated baseline shows this); Fisher-optimal input design and active identification plan the excitation offline; dual control accounts for information implicitly in the cost.

Author action: New Table 1 (what each family measures, its gate signal, whether it isolates the mass–vertical-force coupling, whether it restores identifiability) and a paragraph in the Introduction with the references [Han 2009; Chen et al. 2016; Ioannou and Sun 1996; Narendra and Annaswamy 1987; Mehra 1974; Morelli 2022; Mesbah 2018; Wüest et al. 2019; Burri et al. 2018; Boyacioglu et al. 2023].

----------------------------------------------------------------------
Reviewer#1, Comment 2:

> Equations (8)–(12) treat the external specific force $d$ as constant over the information window. In the EKF, however, $d$ is modeled as a random walk, and in the simulations it includes wind and unmodeled aerodynamic drag. If $d$ varies appreciably over the window, it can absorb changes associated with $T a/m$, and the Schur complement derived for a constant nuisance parameter is no longer the exact marginal Fisher information of the implemented model. The authors should either extend the derivation to the stochastic time-varying disturbance model or quantify the approximation error as a function of window length, disturbance bandwidth, and $q_d$. This point is important because the proposed gate is justified directly through the Fisher-information interpretation.

Author response: Correct. Under the random-walk model the exact marginal information on the inverse mass is a weighted Schur complement,

$$ \frac{\sigma_{\mathrm{eff}}}{r_s} \;=\; t^{\top}\Sigma^{-1}t \;-\; t^{\top}\Sigma^{-1}M\,\bigl(M^{\top}\Sigma^{-1}M\bigr)^{-1}M^{\top}\Sigma^{-1}t, \qquad \Sigma = r_s I + q_d\, C \otimes I_3,\quad [C]_{kl} = \min(k,l) - 1, $$

which reduces to $\sigma/r_s$ for $q_d \to 0$.

Evaluated on logged flight data (new Fig. 14(b)): for the implemented $q_d = 10^{-2}$ the information in a 1-s window is 12% of the constant-force value, and the ratio falls with the window length (0.25–4 s) and with $q_d$ (0–0.1). Two consequences follow.

- Proposition 2 is an optimistic bound for the implemented filter; scaled by this ratio it is of the scale of the empirical across-run spread of the mass under excitation (Fig. 14(a)).
- For the disturbance profiles (steps, sinusoid) and the $q_d$ range evaluated, the gate decision is unaffected: it thresholds the dimensionless $\tilde\sigma$ with margin, and at hover $\sigma_{\mathrm{eff}}$ stays at the noise floor for every $q_d$ tested.

The random-walk variance is the bandwidth parameter of the implemented model; we did not sweep the physical disturbance frequency separately, and the manuscript now says so.

Author action: Derivation in Section III (Eqs. (13)–(15)), new Fig. 14, new paragraph after Proposition 2 in Section III, and the bound-versus-spread comparison in Section VII-I.

----------------------------------------------------------------------
Reviewer#1, Comment 3:

> The quantity $\sigma$ is the dispersion of the thrust vectors $T_k a_k$, not merely the dispersion of the thrust direction. It may become positive through changes in thrust magnitude, changes in direction, or both. Several passages describe $\tilde\sigma$ primarily as a measure of directional excitation, which is incomplete. More importantly, normalizing by $\sum T_k^2$ removes the overall thrust scale, but it also removes information relevant to the actual Cramér–Rao bound. Two maneuvers may have the same $\tilde\sigma$ while providing different absolute Fisher information because of different thrust magnitudes or accelerometer-noise levels. The manuscript should distinguish clearly between the normalized classification signal and the actual information content.

Author response: Correct on both counts. $\sigma$ becomes positive through changes in thrust magnitude, direction, or both; the "directional" wording was imprecise. Normalizing by $\sum_k T_k^2$ removes the thrust scale on purpose (one threshold across maneuvers) but discards information the bound retains: equal $\tilde\sigma$ with different thrust or noise levels means different Fisher information.

Author action: Definition of $\sigma$ rewritten after Eq. (10); text after Eq. (11) states what the normalization discards; $\tilde\sigma$ is now called the classification signal and $\sigma/r_s$ the information content throughout; new Table 6 lists both, plus the one-second Cramér–Rao mass uncertainty, for hover, five speeds, and the probe.

----------------------------------------------------------------------
Reviewer#1, Comment 4:

> The manuscript states that one normalized threshold works across the tested maneuvers. This is useful within the reported simulation environment, but it does not establish that the threshold will transfer across vehicles, thrust-to-weight ratios, IMU noise levels, sample rates, window lengths, or attitude-estimation quality. The authors should provide a threshold-sensitivity study covering at least the information-window duration, dwell time, process noise, measurement noise, and maneuver intensity. It would also be helpful to relate the threshold to an allowable mass-estimation variance through the Cramér–Rao bound rather than selecting it solely from separation between hover and maneuver data.

Author response: Both done.

Threshold vs. bound: near hover $\sum_k T_k^2 \approx N(mg)^2$, so for a window that just opens the gate the Cramér–Rao bound reads (new Eq. (16))

$$ \frac{\mathrm{std}(\hat m)}{m} \;\gtrsim\; \frac{1}{g}\sqrt{\frac{r_s}{\tilde\sigma_{\min}\, N}}\,, $$

which gives 7.6% for one second of data and 2.4% after ten with our parameters. The bound is a floor, not a guarantee: for a one-second window at or below the threshold, even an efficient unbiased estimator could not attain a standard deviation smaller than 7.6%, which is of the scale of the 5–7% hover bias the gate guards against. Under the same ideal constant-force assumptions, the floor crosses the observed 3.3% residual after approximately 5.3 seconds of accumulated admissible data and reaches 2.4% after ten seconds. These figures characterize only the best-case variance permitted by the information; they neither predict nor upper-bound the realized estimation error.

Sensitivity (new Table 7; windy maneuver-to-hover transition of Fig. 7, one parameter at a time, five seeds each, RMS mass error over the hover; reference 2.2% gated vs 5.1% plain):
- Threshold: below 0.009 the gate re-opens on hover excursions (3.4–3.8%); at 0.015 it is erratic (5.1 ± 2.8%); at 0.03–0.06 it holds the mass to 0.4–0.6%. The threshold is limited not by the hover but by what must re-open the gate: with the 0.8-m budget the probe reaches $\tilde\sigma \approx 0.048$, so a threshold of 0.03 would be admissible for the probe and would hold the mass below 1%, but it would exclude the 3.8 m/s flight ($\tilde\sigma = 0.014$) from identification; we therefore retain 0.009 and accept the residual re-openings in windy hover.
- Dwell 0.1–1 s: gate never worse than plain, best at 0.25 s; long dwells make re-openings rare but larger.
- Window: 0.5 s too short (8.8%); 1–2 s hold at 2.2–3.1%.
- Process noise: gate helps for $q_m \le 10^{-5}$, counterproductive at $10^{-4}$; essential at $q_d = 10^{-3}$ (plain 53%, gated 6.9%), unnecessary at $q_d = 10^{-1}$ where the force absorbs everything (plain 1.8%). A loose force model hides the ambiguity instead of resolving it and is correct only if the mass never changes.
- Accelerometer noise 0–0.7 m/s$^2$: gated error stays at 2.2–2.5%; the plain drift itself shrinks (1.9–2.9%), so the margin narrows; at 0.3 m/s$^2$ the two are statistically indistinguishable ($2.5\pm1.1$ against $1.9\pm0.9$).
- Maneuver intensity: unchanged at 5.9–8.4 m/s; at 3 m/s peak $\tilde\sigma$ never exceeds the threshold, neither filter identifies the mass, and both are vulnerable to the hover-onset transient (44% plain, 40 ± 44% gated). This is the regime the active probe exists for.
Transfer to other vehicles and IMU grades is not established by one simulated platform; the paper now says so at the end of Section VII-I. Equation (16) provides a necessary best-case information condition, not a sufficient accuracy guarantee, for selecting a threshold from $N$, $r_s$, and the desired mass uncertainty.

Author action: New Eq. (16) and paragraph in Section III; new Section VII-I with Table 7 (sensitivity), the offline window analysis, and Table 6; operating region stated explicitly (window 1–2 s, dwell about 0.25 s, $q_m \le 10^{-5}$, $q_d \le 10^{-2}$, threshold between hover excursions and probe level); the battery scripts are included with the released code.

======================================================================
Reviewer #2
======================================================================

Reviewer#2, Comment 1:

> The paper could possibly benefit from a comment on the validity of using an EKF wrt. the levels of non-linearity (if local linearization is sufficient), and under what scenarios (rapid motion, rotated frames) it could be less applicable.

Author response: The model is linear in position, velocity, and force; the attitude enters as a known input, not as a state; drag is not estimated. The only nonlinear dependence on the state is $T/m$, whose linearization about $\hat m$ has a relative curvature error $(\delta m/m)^2$: below 1% for a 10% mass error, 18% for the 0.60 kg prior of Fig. 5, which still converges. The matched moving-horizon estimator (full nonlinear window, no local linearization) did not improve on the EKF, so the linearization is not the limiting factor. The scenarios named by the reviewer matter for other reasons: under rapid rotation a latency $\delta t$ between IMU and attitude gives an error of order $g\,\omega\,\delta t$ (0.5 m/s$^2$ at 5 rad/s and 10 ms), a synchronization requirement; and since the force is in the world frame, a body-fixed disturbance (drag at speed) must be followed by the random walk as the attitude turns it, the bandwidth limit already noted in Section II. A body-frame force state or an unscented/iterated update would be the drop-in changes if either dominated.

Author action: New Section IV-B, "Validity of the linearization".

======================================================================
Reviewer #3
======================================================================

Reviewer#3, General comment:

> Summary: The work in this paper addresses the challenge of identifying force and mass parameters in a quadrotor when identifiability requirements are not always met due to operational constraints in hover. The authors approach this challenge by building a reduced order model, characterizing the conditions when identifiability is met, creating a gating process on the filter to prevent propagation of mass data when the mass is not identifiable from the vertical force, and creating motion guidelines to ensure identifiability. Mathematical analysis supports the work, and results are demonstrated with a reasonable fidelity software-in-the-loop system. The work is of high interest to the community, the approach to the solution is thoughtful and novel, and the results are all sound. Appropriate context is given for the work and the relevant literature. With some minor revisions, the work is suitable for publication.

Author response: We thank the reviewer for the careful reading and the positive assessment; all points are addressed below.

----------------------------------------------------------------------
Reviewer#3, Key point:

> The authors in many places, including the title, interchangeably use the terms "observability" and "identifiability". The work in this paper specifically addresses identifiability, not observability. The two concepts are closely linked, but they are not the same. The terms "observability" should be replaced with "identifiability".
> For work addressing observability of mass in this type of system, the authors may wish to refer to: B. Boyacioglu, D. Sandursky, and K. A. Morgansen, "Nonlinear estimation of rigid body inertial parameters," 2023 AIAA Scitech.

Author response: Agreed. The quantity analyzed is the identifiability of the parameters $(m, d)$ in the accelerometer regressor; observability of the augmented state coincides with it only because position and velocity are measured directly.

Author action: Title changed to "Identifiability-Aware Virtual Force Sensing and Self-Calibration for Quadrotors"; "observability"/"observable" replaced by "identifiability"/"identifiable" throughout (abstract, keywords, section titles, Table 2, figure captions and figure legends, conclusion); "observability" kept only for the nonlinear observability of the augmented state in Section III, where a sentence now states the distinction and cites [Boyacioglu et al. 2023].

----------------------------------------------------------------------
Reviewer#3, Minor point 1:

> In the introduction (p. 1, line 46), one should note that mass is not a force. Mass with gravity is a force.

Author response and action: Reworded in the abstract, the Introduction, and Section II: a mass error, which enters the reading through the thrust-to-weight ratio, is what is indistinguishable from a vertical force; Section II now says "the weight of an added mass".

----------------------------------------------------------------------
Reviewer#3, Minor point 2:

> The result on page 4 in Equation (12) could be stated as a Proposition as it has been mathematically proved here.

Author response and action: Done; it is now Proposition 2 (Cramér–Rao bound on the mass) with its proof.

----------------------------------------------------------------------
Reviewer#3, Minor point 3:

> On p. 6, lines 17-20, comments are made about smoothing the estimate. Please comment on why the result of the estimator is not providing a sufficiently smoothed result.

Author response and action: A paragraph in Section V explains it. The filter's $q_d$ is set for the bandwidth of the force reading, so its output retains sample-to-sample noise at 100 Hz that the controller would pass into the thrust command; the moving average is a low-pass on the actuation path, tunable independently of the estimation bandwidth (0.12 s for the force; 2 s for the mass, which sets the hover equilibrium of the prediction model).

----------------------------------------------------------------------
Reviewer#3, Minor point 4:

> On page 6, Section V.B, the reader would benefit from some of the recent(ish) work on optimal excitation for identifiability, particularly in flight systems. One suggestion would be E. A. Morelli, "Determining aircraft moments of inertia from flight test data," AIAA Journal of Guidance, Control, and Dynamics, Jan. 2022, 45(1):4-14.

Author response and action: [Morelli 2022] is now cited in Section V-B, together with [Mehra 1974], as the input-design problem we do not solve in continuous form.

----------------------------------------------------------------------
Reviewer#3, Minor point 5:

> Page 9, Figure 8, a somewhat longer timeline would be helpful to show a bit more of the behavior of the mass estimate after the active probe region. Maybe also a bit more within the active probe region to show that the value is staying at the nominal value rather than passing through and continuing on.

Author response and action: Fig. 8 was regenerated from 50-s runs (previously 35 s), showing 34 s of hover after the probe is withdrawn with the mass held at the identified value. In doing so we found that, in the submitted version, the passive runs of Fig. 8 had been flown without wind and the active runs with it, and that the 0.4-m probe budget left the probe's $\tilde\sigma$ at the gate threshold. Both experiments (Figs. 8 and 9) were therefore re-run under identical wind-free conditions with the budget raised to 0.8 m and an explicit, empirically chosen withdrawal rule (15 s of gate-open time; the constant-force bound of Eq. (16) for the information accumulated in that time is about 1%, the order of the residual observed). Results: recovery from the 0.70-kg prior to $0.4 \pm 0.2$% (N = 5) with the most informative probe; all shapes at this budget exceed the threshold and the achieved $\tilde\sigma$ (0.033 to 0.048) sets the settling time (14.3 to 8.3 s, $p = 0.004$), which is now what Fig. 9 reports. Sections V-B, VII-D, VII-E, VII-G and Tables 3, 6 were updated accordingly.

----------------------------------------------------------------------
Reviewer#3, Typographical and formatting notes:

> Sentences that start with "Fig." should be changed to "Figure". (e.g., p. 2, line 53)
> Page 3, line 6, "Section VIII concludes." should perhaps be changed to "Section VIII concludes the paper."
> Page 3, line 30, "body rate" is more appropriately denoted "body attitude rate".
> Page 4, line 23, remove the comma after "separated"
> Page 4, line 31, add a comma after "mass"
> Page 4, line 40, capitalize Newtons
> Page 4, line 47, the term "Dividing out" could be changed to "Normalizing by"
> Page 7, line 1, Table 2 should appear on the page where it was first mentioned (or the page immediately after). Ideally, page 6 rather than page 7.
> Page 8, Figure 4, the font size in figures should be roughly the same as the caption font size. Potentially the headers on the plots could be incorporated into the caption as well.
> Page 11, Figure 13, same comments as for Figure 4.

Author action: No sentence now begins with a figure citation: the affected sentences were rephrased so the reference falls mid-sentence, where the IEEE Editorial Style Manual prescribes the abbreviated "Fig."; this satisfies the request without deviating from the house style; "Section VIII concludes the paper."; "body attitude rate (body rate for short)" at the definition of $\omega$; comma after "separated" removed; comma after "mass" added; "newtons squared" kept in lowercase, as SI prescribes for unit names spelled out (the italic $N$ is reserved for the window length, so we did not switch to the symbol either); we hope the reviewer accepts this; "Dividing out" changed to "Normalizing by"; Table 2 now appears on the page of its first mention, and Table 3 was moved next to its first mention as well; Figs. 4 and 13 regenerated as single-column three-row stacks with fonts at caption size; the per-panel headers of Fig. 4 carry only the panel label, with the fitted slopes in the legend, caption, and text.

======================================================================
Reviewer #4
======================================================================

Reviewer#4, Comment 1:

> Report the compute platform for the "<1 ms per step at 100 Hz" real-time claim, and if it's not an embedded/flight-representative processor, say so.

Author response: The timings were measured on a laptop-class processor (Intel Core Ultra 9 275HX, one core per solver), not on an embedded flight computer; the method has not been validated on embedded hardware, and we do not claim it.

Author action: Platform stated in Section V-A where the solve time is first given, and referenced again in Section VII-F.

----------------------------------------------------------------------
Reviewer#4, Comment 2:

> Consider adding a sentence on why the injected-error ranges in Section VII-H (0.4 m/s$^2$ accel bias, 10% scale/thrust error) were chosen.

Author response: They bracket what a real platform presents before any in-field calibration: the bias range covers the uncalibrated initial offset of consumer MEMS accelerometers (tens of mg, a few tenths of m/s$^2$), the scale-factor range covers their sensitivity tolerance of a few percent with a wide margin, and the thrust range covers the change of the thrust map over a flight from battery-voltage sag and propeller wear, of the order of 10%.

Author action: Sentence added in Section VII-H.

----------------------------------------------------------------------
Reviewer#4, Comment 3:

> Clarify the d_z correlation degradation with speed (Table 3).

Author response: The ground-truth channel contains only the applied wind, whereas the reading also contains the lumped aerodynamic drag ($c = 0$ in the estimator). The figure-eight has a vertical velocity component, so the lumped drag acquires a vertical part that grows with speed (its driver $v_z\|v\|$ rises from 0.02 to 14 m$^2$/s$^2$ RMS across the battery); it is uncorrelated with the applied steps, and the vertical steps are half the size of the horizontal ones (0.8 N against 1.5 N on $x$), so the same uncorrelated content costs more correlation on that axis. The absolute vertical error moves the other way, from 0.62 N in hover, where the mass–force ambiguity biases the plain filter's $d_z$ (a bias the correlation does not see), to 0.29 N at 10.2 m/s; the dip to 0.91 at 6.1 m/s is comparable to the run-to-run spread ($\pm 0.02$).

Author action: Paragraph added in Section VII-A, and Table 4 (Table 3 of the submitted version) moved next to it.

======================================================================
Reviewer #5
======================================================================

Reviewer#5, General comment:

> This paper addresses an important problem: obstacle calibration suffers from an observability issue. Specifically, the measurement requires the vehicle mass information, but the mass is unobservable in near-hover conditions because it is difficult to distinguish the effect of mass from the vertical external force. Therefore, applying a standard Extended Kalman Filter (EKF) may introduce disturbances into the mass estimation. To address this issue, the authors propose a method entitled "Observability-Aware Virtual Force Sensing and Self-Calibration for Quadrotors." I believe this problem is meaningful and exists in real-world scenarios. However, although I have provided revision suggestions previously, several aspects are still not clearly explained and require further clarification.

Author response: We thank the reviewer for the assessment; each of the eight questions is answered below.

----------------------------------------------------------------------
Reviewer#5, Question 1:

> In Innovation 1, the authors state that "the mass and the vertical external force are not jointly observable in near-hover flight, and we derive an online identifiability measure." This description seems more like an external disturbance estimation method. What is the fundamental difference between the proposed method and existing approaches using an Extended State Observer (ESO) or an Unknown Disturbance Observer (UDO)?

Author response: The force reading itself is not different: it is the accelerometer residual, the same signal an ESO or a disturbance observer builds its disturbance state on. Those observers assume the model, and hence the mass that scales the residual, known; they have no notion of whether the mass is currently separable from the vertical force, and no way to act when it is not. Our contribution is the calibration layer around the residual: (i) an online, closed-form test of that separability (the Schur complement, i.e. the marginal Fisher information on the mass), (ii) a gate that freezes the mass when the test fails, and (iii) an excitation triggered and shaped by the same test.

Author action: New Table 1 and the accompanying paragraph in the Introduction, which now states explicitly that the disturbance state of an ESO/UDO is the same residual our reading is built on and that what we add is the calibration layer.

----------------------------------------------------------------------
Reviewer#5, Question 2:

> In Innovation 2, the authors state that the estimation is "protected when unobservable so the reading is protected during hover." How is unobservability quantitatively defined in this work? What criterion is used to determine whether the system is in an unobservable condition?

Author response: The criterion is the normalized Schur complement of Eq. (11), computed over a 1-s sliding window from the known thrust and attitude, falling below a threshold: $\tilde\sigma < \tilde\sigma_{\min} = 0.009$ (Eq. (18)). Eq. (11) is exactly zero in stationary hover (Proposition 1) and grows with the dispersion of the thrust vector; Table 6 reports its value per regime and Section VII-I its sensitivity.

Author action: The contribution bullet and Section IV-C now state the criterion explicitly ("the mass is declared unidentifiable, quantitatively, when $\tilde\sigma$ falls below $\tilde\sigma_{\min}$"). Equation (16) states the corresponding Cramér–Rao lower bound as a necessary best-case information condition, not an accuracy guarantee.

----------------------------------------------------------------------
Reviewer#5, Question 3:

> In Equation (1), the dynamic model contains many specific physical parameters. How can the authors guarantee the accuracy of the calibration or identification of each parameter?

Author response: Few of them enter the reading. $g$ is known; the drag coefficient is not identified but set to zero, so any drag is lumped into the force and read as part of it; $\tau_{rc}$ enters only the controller's prediction model; the mass is estimated. The single calibration the reading depends on is the thrust map that gives $T$; its error is characterized in Section VII-H: a 10% thrust mismatch couples almost one-to-one into the mass estimate and barely into the force.

Author action: A paragraph at the end of Section II-B now lists, parameter by parameter, what is assumed, lumped, estimated, or calibrated, with the pointer to Section VII-H.

----------------------------------------------------------------------
Reviewer#5, Question 4:

> In Equation (12), the authors state that the inverse mass parameter β has boundedness and emphasize this as one of the contributions in Innovation 1. However, this property is not experimentally validated; it is only briefly mentioned in the text. Could the authors provide experimental verification to support this claim?

Author response: Eq. (12) is not a boundedness property of $\beta$; it is the Cramér–Rao lower bound on the variance of any unbiased estimate of $\beta$, i.e. the statement that the Schur complement is the marginal Fisher information on the mass. We agree that it deserved a clearer statement and an experimental check.

Author action: Eq. (12) is now Proposition 2 with its proof; new Section VII-I and Fig. 14(a) compare the bound with the empirical across-run spread of the mass estimate at six flight speeds (the bound is of the scale of the spread under excitation once the random-walk force model is accounted for, Section III), and Table 6 reports the bound per regime.

----------------------------------------------------------------------
Reviewer#5, Question 5:

> At the end of Page 5, what exactly does the term "probe" refer to? I could not clearly understand its meaning. Is it a physical sensor or another type of measurement device?

Author response: Not a sensor. The probe is a small, bounded excursion added to the position reference of the controller (Eq. (22): a lateral circle and/or a vertical bob of amplitude within a budget $\rho$), injected only while the mass is needed and unidentifiable, and withdrawn afterwards.

Author action: Defined at its first use in the Introduction ("a small bounded excursion of the position reference (not a sensor)") and in Section V-B.

----------------------------------------------------------------------
Reviewer#5, Question 6:

> Regarding the optimization function in Equation (16), does the objective function require normalization? Since different terms may have significantly different magnitudes, one dominant term may overwhelm the others, making the variations of smaller terms insignificant.

Author response: The weights are the normalization. With the values of Table 3, a position error of 0.1 m, a velocity error of 0.25 m/s, an attitude error of 0.3 rad and a thrust deviation of 2.5 N each contribute about 1.8 to the stage cost, so no term dominates at the errors the controller operates at.

Author action: Sentence added after Eq. (20) (Eq. (16) of the submitted version) stating this.

----------------------------------------------------------------------
Reviewer#5, Question 7:

> Regarding Equation (16), the optimization objective is provided, but the final controller expression is not explicitly given. Is this formulation complete and reasonable, considering that the section is titled "V. CLOSED-LOOP USE: CONTROL AND ACTIVE CALIBRATION"?

Author response: The controller is the receding-horizon law: at each step the problem is solved from the current state, mass, and force estimates, the first input of the optimal sequence is applied, and the horizon shifts.

Author action: New Eq. (21), $u_k = u_0^{\star}(\hat x_k, \hat m, \hat d_k)$, with the sentence describing the receding-horizon rule, in Section V-A.

----------------------------------------------------------------------
Reviewer#5, Question 8:

> If all physical parameters are accurately known, according to the fundamental dynamic equation, would the difference between the kinetic energy generated by the robot output and the kinetic energy received by the robot directly represent the disturbance magnitude? Please clarify the conceptual difference between the proposed "virtual force sensing" approach and this intuitive interpretation.

Author response: An energy balance can indeed be formed, but it is a scalar (it loses the direction of the force), it is an integral (the force enters as $\int d \cdot v\,dt$, so it lags), and it vanishes wherever the velocity does, which is precisely the hover where the force matters most. The specific-force reading is a vector, instantaneous, and independent of the velocity.

Author action: Paragraph added in Section III after the discussion of the algebraic measurement.
