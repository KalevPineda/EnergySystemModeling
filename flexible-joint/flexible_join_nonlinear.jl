# Importar paquetes necesarios
using DifferentialEquations
using Plots


#Configurar el backed grafico 
try 
  gr()
  println("✅ Backend GR configurado")
catch e
  println("❌ Error configurando GR: $e")
end


# --- 1. Definición de Parámetros del Sistema ---
const m = 1.1        # Masa del brazo (kg)
const g = 9.81       # Aceleración de la gravedad (m/s^2)
const l = 0.4        # Distancia al centro de masa (m)
const Jm = 1.0       # Inercia del motor (kg*m^2)
const Jl = 0.0587    # Inercia del brazo (kg*m^2)
const K = 50.0       # Constante de rigidez del resorte (N*m/rad)
const Bm = 1.0       # Coeficiente de fricción del motor (N*m*s/rad)
const Bl = 1.0       # Coeficiente de fricción del brazo (N*m*s/rad)
const MGL = m * g * l # Término gravitacional precalculado

# El vector de parámetros 'p' se pasará a las funciones de la dinámica.
const p = [Jm, Jl, K, Bm, Bl, MGL]

# 2. Definir la excitación del motor (torque)
# Usaremos un "chirp" para barrer frecuencias de torque de 0 a 15 Hz en 100 segundos
# Esto nos permite ver las resonancias del sistema en el eje bajo excitación de torque

#function motor_torque(t)
    # Ejemplo: un torque sinusoidal que varía con el tiempo
#    return 5 * sin(2π * t / 10) + 10 # Un torque que varía entre 5 y 15
#end

# Para un toque constante
function motor_torque(t)
    return 0.5
end

# --- 3. Definición de las Ecuaciones de Movimiento ---

# El estado del sistema se define como u = [ψ, θ, dψ/dt, dθ/dt]
# Por lo tanto, du = [dψ/dt, dθ/dt, d²ψ/dt², d²θ/dt²]

# Modelo NO LINEAL
function nonlinear_system!(du, u, p, t)
    # Desempaquetar estados y parámetros para mayor claridad
    ψ, θ, dψ, dθ = u
    Jm, Jl, K, Bm, Bl, MGL = p
    τ = motor_torque(t)

    # du[1] = dψ/dt
    du[1] = dψ
    # du[2] = dθ/dt
    du[2] = dθ
    # du[3] = d²ψ/dt² (de la Ecuación 1)
    du[3] = (1/Jm) * (τ - Bm*dψ - K*(ψ - θ))
    # du[4] = d²θ/dt² (de la Ecuación 2, con sin(θ))
    du[4] = (1/Jl) * (K*(ψ - θ) - Bl*dθ - MGL*sin(θ))
end

# --- 4. Configuración y Ejecución de la Simulación ---

# Condición inicial: [ψ(0), θ(0), dψ(0)/dt, dθ(0)/dt]
# Soltamos el brazo desde 45 grados (π/4 rad) sin velocidad inicial.
# El motor está en su posición de reposo.
u0 = [0.0, π/4, 0.0, 0.0]
#u0 = [0.0, 0.0 ,0.0, 0.0]

# Intervalo de tiempo para la simulación
tspan = (0.0, 25.0)

# Crear los problemas de EDO (Ecuaciones Diferenciales Ordinarias)
prob_nonlinear = ODEProblem(nonlinear_system!, u0, tspan, p)

# Resolver las EDOs
println("Resolviendo el sistema no lineal...")
sol_nonlinear = solve(prob_nonlinear, Tsit5(), reltol=1e-8, abstol=1e-8)
println("Resolviendo el sistema lineal...")
sol_linear = solve(prob_linear, Tsit5(), reltol=1e-8, abstol=1e-8)
println("Simulación completada.")
println("Calculando graficas.")

# --- 4. Graficación y Comparación de Resultados ---

# Extraer los resultados para el ángulo del brazo (θ), que es el segundo estado (u[2])
t_nl = sol_nonlinear.t
θ_nl = [u[2] for u in sol_nonlinear.u]
ψ_nl = [u[1] for u in sol_nonlinear.u]
dθ_nl = [u[4] for u in sol_nonlinear.u]
dψ_nl = [u[3] for u in sol_nonlinear.u]

t_l = sol_linear.t
θ_l = [u[2] for u in sol_linear.u]
ψ_l = [u[1] for u in sol_linear.u]
dθ_l = [u[4] for u in sol_linear.u]
dψ_l = [u[3] for u in sol_linear.u]

# du[1] = dψ/dt
# du[2] = dθ/dt
# du[3] = d²ψ/dt² (de la Ecuación 1)
# du[4] = d²θ/dt² (de la Ecuación 2, con sin(θ))

plot_velocidades = plot(t_nl, dψ_nl,
                        label="Velocidad Motor (dψ/dt)",
                        xlabel="Tiempo (s)",
                        ylabel="Velocidad Angular (rad/s)",
                        title="Velocidades Angulares del Sistema",
                        legend=:topright,
                        linewidth=2,
                        size=(1920, 1080)) # Tamaño Full HD
plot!(plot_velocidades, t_nl, dθ_nl,
      label="Velocidad Carga (dθ/dt)",
      linewidth=2,
      linestyle=:dash) # Línea punteada para diferenciar

# 2. Diagramas de Fase (dψ/dt vs ψ y dθ/dt vs θ)
# Un diagrama de fase muestra la relación entre una variable de estado y su derivada.
# Las oscilaciones o vibraciones se ven como órbitas cerradas o espirales.

# Diagrama de Fase del Motor
plot_fase_motor = plot(ψ_nl, dψ_nl,
                       label="", # No se necesita etiqueta en el plot principal
                       xlabel="Posición Motor (ψ, rad)",
                       ylabel="Velocidad Motor (dψ/dt, rad/s)",
                       title="Diagrama de Fase del Motor",
                       legend=false,
                       linewidth=1.5,
                       color=:blue,
                       size=(1920, 1080)) # Tamaño Full HD
plot!(plot_fase_motor, [ψ[1]], [dψ[1]], seriestype=:scatter, markersize=5, color=:red, label="Inicio") # Punto de inicio

# Diagrama de Fase de la Carga
plot_fase_carga = plot(θ, dθ,
                       label="",
                       xlabel="Posición Carga (θ, rad)",
                       ylabel="Velocidad Carga (dθ/dt, rad/s)",
                       title="Diagrama de Fase de la Carga",
                       legend=false,
                       linewidth=1.5,
                       color=:green,
                       size=(1920, 1080)) # Tamaño Full HD
plot!(plot_fase_carga, [θ[1]], [dθ[1]], seriestype=:scatter, markersize=5, color=:red, label="Inicio") # Punto de inicio


# ---------------------
# Crear la gráfica para el ángulo del brazo θ
plot_theta = plot(t_nl, θ_nl,
    label="No Lineal (θ)",
    linewidth=2,
    xlabel="Tiempo (s)",
    ylabel="Ángulo del Brazo θ (rad)",
    title="Comparación de Respuestas: Modelo Lineal vs. No Lineal",
    legend=:topright)

plot!(plot_theta, t_l, θ_l,
    label="Lineal (θ)",
    linewidth=2,
    linestyle=:dash)
display(plot_theta)

#savefig(plot_theta,"desplazamiento_brazo_nl_vs_l.png")
println("Enter, para la siguiente grafica ...")
readline()

# Crear la gráfica para el ángulo del motor ψ
plot_psi = plot(t_nl, ψ_nl,
    label="No Lineal (ψ)",
    linewidth=2,
    xlabel="Tiempo (s)",
    ylabel="Ángulo del Motor ψ (rad)",
    title="Respuesta del Ángulo del Motor",
    legend=:topright)
    
plot!(plot_psi, t_l, ψ_l,
    label="Lineal (ψ)",
    linewidth=2,
    linestyle=:dash)
display(plot_psi)

#savefig(plot_psi, "desplazamiento_rotor_nl_vs_l.png")
println("Enter, para la siguiente grafica")
readline()


# Crear la gráfica para el ángulo del brazo θ
plot_angle = plot(t_nl, θ_nl,
    label="Ángulo del Brazo θ (rad)",
    linewidth=2,
    xlabel="Tiempo (s)",
    ylabel="Ángulo del Brazo (rad)",
    title="Desplazamiento de θ y ψ (No lineal)",
    legend=:topright)

plot!(plot_angle, t_nl, ψ_nl,
    label="Ángulo del Brazo ψ (rad)",
    linewidth=2,
    linestyle=:dash)

display(plot_angle)    #savefig(plot_angle, "angle.png")
#savefig(plot_angle, "angle.svg")
print("Enter, para continuar")
readline()

# Mostrar ambas gráficas en una sola ventana
combinar =plot(plot_theta, plot_psi, layout=(2,1))
display(combinar)
println("Enter, para salir ...")
readline()

