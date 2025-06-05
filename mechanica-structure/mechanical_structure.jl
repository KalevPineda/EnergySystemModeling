# ----------------------------------------------------
# VIBRATION CONTROL IN A BUILDING LIKE STRUCTURE
# Simulación del modelo de 3 GDL
# Autores: Kalev Pineda, Josue Sarate
# ----------------------------------------------------

# 1. Importar paquetes necesarios
using DifferentialEquations
using Plots
using LinearAlgebra

#Configurar el backed grafico                                                     
try                                                                               
    gr()                                                                            
    println("✅ Backend GR configurado")                                            
catch e                                                                           
    println("❌ Error configurando GR: $e")                                         
end                                                                               
     

# 2. Definir los parámetros del sistema (de la Tabla 1 del paper)
m1 = 1.018   # kg
m2 = 1.001   # kg
m3 = 2.187   # kg  (Usamos el m3 de la estructura sin TMD)

k1 = 897.0277 # N/m
k2 = 933.3893 # N/m
k3 = 888.2334 # N/m

c1 = 0.1233  # Ns/m
c2 = 0.3345  # Ns/m
c3 = 1.8977  # Ns/m

# 3. Ensamblar las matrices del sistema
M = diagm([m1, m2, m3])
C = [c1+c2  -c2    0;
     -c2  c2+c3  -c3;
      0    -c3    c3]
K = [k1+k2  -k2    0;
     -k2  k2+k3  -k3;
      0    -k3    k3]

# Inversa de la matriz de masa (para eficiencia)
M_inv = inv(M)
e3 = [1.0, 1.0, 1.0] # Vector de influencia

# 4. Definir la excitación del suelo (aceleración z_ddot)
# Usaremos un "chirp" para barrer frecuencias de 0 a 15 Hz en 100 segundos
# Esto nos permite ver las resonancias del sistema
f_start = 0.0 # Hz
f_end = 6.0 # Hz
t_final = 5.0
function ground_acceleration(t)
    # A(t) * sin(phi(t))
    amplitude = 1.0 # m/s^2
    current_freq = f_start + (f_end - f_start) * (t / t_final)
    return amplitude * sin(2.0 * π * current_freq * t)
end

# 5. Definir la función de la EDO en forma de espacio de estados
# El estado u = [x1, x2, x3, dx1, dx2, dx3]
# du es la derivada del estado [dx1, dx2, dx3, ddx1, ddx2, ddx3]
function three_story_building!(du, u, p, t)
    # Desempacar parámetros y estado
    M, C, K, M_inv, e3 = p
    x = @view u[1:3]     # Desplazamientos relativos
    x_dot = @view u[4:6] # Velocidades relativas

    # Calcular la fuerza de excitación en el tiempo t
    z_ddot_t = ground_acceleration(t)
    F_t = -M * e3 * z_ddot_t

    # Ecuación de movimiento: M*x_ddot + C*x_dot + K*x = F
    # Despejar x_ddot: x_ddot = M_inv * (F - C*x_dot - K*x)
    x_ddot = M_inv * (F_t - C * x_dot - K * x)

    # Asignar las derivadas al vector du
    du[1:3] .= x_dot
    du[4:6] .= x_ddot
end

# 6. Configurar y resolver el problema de valor inicial
u0 = zeros(6)             # Condiciones iniciales: reposo
tspan = (0.0, t_final)    # Intervalo de tiempo de la simulación
p = (M, C, K, M_inv, e3)  # Parámetros para la función

prob = ODEProblem(three_story_building!, u0, tspan, p)
sol = solve(prob, Tsit5(), reltol=1e-6, abstol=1e-6)

# 7. Post-procesamiento y graficación

# Extraer tiempo y soluciones
t = sol.t
x1 = [u[1] for u in sol.u]
x2 = [u[2] for u in sol.u]
x3 = [u[3] for u in sol.u]

dx1 = [u[4] for u in sol.u]
dx2 = [u[5] for u in sol.u]
dx3 = [u[6] for u in sol.u]

# --- Gráficas ---

# Gráfica 1: Aceleración del suelo (Excitación)
z_ddot_plot = plot(t, ground_acceleration.(t),
    title="Aceleración del Suelo (Excitación)",
    xlabel="Tiempo (s)",
    ylabel="Aceleración (m/s²)",
    label="ẍ(t)",
    legend=:topright,
    size=(1920,1080),
    lw=2)

# Gráfica 2: Desplazamientos relativos de cada piso
displacement_plot = plot(t, [x1, x2, x3],
    title="Desplazamientos Relativos de los Pisos",
    xlabel="Tiempo (s)",
    ylabel="Desplazamiento (m)",
    label=["Piso 1 (x₁)" "Piso 2 (x₂)" "Piso 3 (x₃)"],
    legend=:topright,
    size=(1920,1080),
    lw=2)

# Gráfica 3: Velocidades relativas de cada piso
velocity_plot = plot(t, [dx1, dx2, dx3],
    title="Velocidades Relativas de los Pisos",
    xlabel="Tiempo (s)",
    ylabel="Velocidad (m/s)",
    label=["Piso 1 (ẋ₁)" "Piso 2 (ẋ₂)" "Piso 3 (ẋ₃)"],
    legend=:topright,
    size=(1920,1080),
    lw=2)

# Gráfica 4: Diagrama de fase para el tercer piso (el más crítico)
phase_plot_3 = plot(x3, dx3,
    title="Diagrama de Fase (Piso 3)",
    xlabel="Desplazamiento x₃ (m)",
    ylabel="Velocidad ẋ₃ (m/s)",
    label="Trayectoria",
    legend=:bottomright,
    size=(1920,1080),
    lw=1)
# Combinar todas las gráficas en una sola figura
#combine = plot(z_ddot_plot, displacement_plot, velocity_plot, phase_plot_3, layout=(2, 2), size=(1000, 800))
# Mostrar las gráficas por separado
display(z_ddot_plot)
#savefig(z_ddot_plot,"funcion_de_exitacion.png")
print("Presiona Enter para la siguiente grafica")
readline()
display(displacement_plot)
#savefig(displacement_plot,"desplazamiento.png")
print("Presiona Enter para la siguiente grafica")
readline()
display(velocity_plot)
#savefig(velocity_plot,"velocidad.png")

print("Presiona Enter para la siguiente grafica")
readline()
display(phase_plot_3)
#savefig(phase_plot_3,"diagrama_fase.png")

print("Presiona Enter para la siguiente salir")
readline()