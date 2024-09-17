clear
clc

s = tf('s') % laplace variable

w0 = 20; % cutoff frequency

G = w0/(s+w0)


figure(1); clf
bode(G)
w_in = 0.43; % inpout sine wave frequency rad/s

hold on
semilogy(w_in*ones(1,10),linspace(-90,0,10))% plot input frequency on phase plot



G_jw = evalfr(G,1i*w_in) % evaluate transfer function at input frequency


phase_lag = angle(G_jw) % phase angle at input frequency

time_lag = abs(phase_lag)/w_in

% simulate filter output for sine wave input
t = 0:.01:30; % time array for simulation
Amp = 1; %amplitude of sine wave input
sig_in = Amp*sin(w_in*t);
filt_out = lsim(G,sig_in,t);


figure(2); clf
plot(t,sig_in,'--k',t,filt_out,'-r','linewidth',2)