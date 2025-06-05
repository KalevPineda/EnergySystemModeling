using DifferentialEquations, Plots

#Configurar el backed grafico 
try 
  gr()
  println("✅ Backend GR configurado")
catch e
  println("❌ Error configurando GR: $e")
end


# Estructura para los parámetros del péndulo
struct PendulumParams
    m::Float64    # masa (kg)
    l::Float64    # longitud (m)
    g::Float64    # gravedad (m/s²)
    b::Float64    # coeficiente de amortiguamiento
    T::Float64    # torque externo (N⋅m)
end

# Función que define el sistema de ecuaciones diferenciales
function pendulum_dynamics!(du, u, params, t)
    θ, θ_dot = u
    m, l, g, b, T = params.m, params.l, params.g, params.b, params.T
    
    # Sistema de ecuaciones de primer orden
    du[1] = θ_dot                                    # dθ/dt = θ̇
    du[2] = (T - m*g*l*sin(θ) - b*θ_dot)/(m*l^2)   # dθ̇/dt = θ̈
end

# Función para resolver el péndulo
function solve_pendulum(params::PendulumParams, initial_conditions, time_span)
    # initial_conditions = [θ₀, θ̇₀]
    # time_span = (t_inicial, t_final)
    
    prob = ODEProblem(pendulum_dynamics!, initial_conditions, time_span, params)
    sol = solve(prob, Tsit5(), reltol=1e-8, abstol=1e-8)
    
    return sol
end

# Función para graficar resultados
function plot_pendulum_solution(sol, title_text="Péndulo Simple"; save_plots=false, filename="pendulum")
  """
  Esta funcion permite obtener graficas a partir de la solucion numerica obtenida del solver
  @sol: solucion numerica de nuestro sistema.
  @title_text: Titulo del grafico (modificable para cada caso)
  @save_plots: los graficos deben guardarce 
  @filename: nombre del archivo
  @return: en caso de que save_plots sea false, devolvera un grafico combinado, en caso contrario
  imprime en nombre de los archivos creados.
  """
    t = sol.t
    θ = [u[1] for u in sol.u]
    θ_dot = [u[2] for u in sol.u]
    
    p1 = plot(t, θ, label="θ(t)", xlabel="Tiempo (s)", ylabel="Ángulo (rad)", 
              title=title_text, linewidth=2)
    
    p2 = plot(t, θ_dot, label="θ̇(t)", xlabel="Tiempo (s)", ylabel="Velocidad angular (rad/s)", 
              linewidth=2, color=:red)
    
    p3 = plot(θ, θ_dot, label="Espacio de fases", xlabel="θ (rad)", ylabel="θ̇ (rad/s)", 
              linewidth=2, color=:green)
    
    #Mostrar graficas
    display(p1)
    println("Enter para ver la siguiente grafica ...")
    readline()
    
    display(p2)
    println("Enter para ver la siguiente grafica ...")
    readline()

    display(p3)
    println("Enter para continuar ...")
    readline()

    combined_plot = plot(p1, p2, p3, layout=(3,1), size=(800,600))
    #Preguntar si deseamos guardar las graficas, en caso de que no lo definimos
    if save_plots == false
      print("\n💾 ¿Guardar gráficas? (s/N): ")
      response = strip(readline())
      if lowercase(response) in ["s", "si", "y", "yes"]
        print("\n💾 ¿Filename?: ")
        filename = strip(readline())
        save_plots = true
      end 
    end 

    # Guardar gráficas si se solicita
    if save_plots
        # Guardar gráfica combinada
        savefig(combined_plot, "$(filename)_combined.png")
        
        # Guardar gráficas individuales
        savefig(p1, "$(filename)_angle.png")
        savefig(p2, "$(filename)_velocity.png") 
        savefig(p3, "$(filename)_phase.png")
        
        println("Gráficas guardadas como:")
        println("- $(filename)_combined.png")
        println("- $(filename)_angle.png")
        println("- $(filename)_velocity.png")
        println("- $(filename)_phase.png")
    end
    
    return combined_plot
end

# Fin de la parte de modelacion, ahora se implementan las funciones de graficacion




# Ejemplo de como utlizar el modelado
function ejemplo_basico()
    # Definir parámetros del péndulo
    params = PendulumParams(
        1.0,    # masa = 1 kg
        1.0,    # longitud = 1 m
        9.81,   # gravedad = 9.81 m/s²
        0.1,    # amortiguamiento = 0.1
        0.0     # sin torque externo
    )
    
    # Condiciones iniciales: θ₀ = π/4 rad, θ̇₀ = 0 rad/s
    initial_conditions = [π/4, 0.0]
    
    # Resolver para 10 segundos
    time_span = (0.0, 10.0)
    
    sol = solve_pendulum(params, initial_conditions, time_span)
    
    # Graficar resultados
    plot_pendulum_solution(sol, "Péndulo Simple con Amortiguamiento")
end

# Función para comparar casos diferentes
function compare_cases(; save_comparison=false, filename="pendulum_comparison")
  """
  Funcion para comparar diversos casos, con energias conservativas, no conservativas
  y con parametros extremos.
    @save_comparison: la grafica se debe guardar? 
  """
    # Caso 1: Sin amortiguamiento
    params1 = PendulumParams(2.85, 1.0, 9.81, 0.0, 16.0)
    
    # Caso 2: Con amortiguamiento
    #params2 = PendulumParams(1.0, 1.0, 9.81, 0.5, 0.0)
    params2 = PendulumParams(2.85, 1.0, 9.81, 0.65, 16.0)
    
    # Caso 3: Con torque externo
    #params3 = PendulumParams(1.0, 1.0, 9.81, 0.1, 1.0)
    params3 = PendulumParams(2.85, 1.0, 9.81, 0.65, 20.0)
    
    initial_conditions = [0.0, 0.0]
    time_span = (0.0, 50.0)
    
    sol1 = solve_pendulum(params1, initial_conditions, time_span)
    sol2 = solve_pendulum(params2, initial_conditions, time_span)
    sol3 = solve_pendulum(params3, initial_conditions, time_span)
    
    t = 0:0.1:50
    θ1 = [sol1(ti)[1] for ti in t]
    θ2 = [sol2(ti)[1] for ti in t]
    θ3 = [sol3(ti)[1] for ti in t]
    
    comparison_plot = plot(t, [θ1 θ2 θ3], 
         label=["Sin amortiguamiento" "Con amortiguamiento" "Con torque externo"],
         xlabel="Tiempo (s)", ylabel="θ (rad)",
         title="Comparación de diferentes casos del péndulo",
         linewidth=2, size=(1920,1080))
    
    # Guardar comparación si se solicita
    full_filename = filename*"_comparison.png"
    if save_comparison
        #savefig(comparison_plot, "pendulum_comparison.png")
        savefig(comparison_plot, full_filename)
        println("Gráfica de comparación guardada como: ($full_filename)")
    end
    
    return comparison_plot
end

# Función para análisis de estabilidad (pequeñas oscilaciones)
function linearized_analysis(params::PendulumParams)
    m, l, g, b = params.m, params.l, params.g, params.b
    
    # Coeficientes de la ecuación característica: s² + (b/ml²)s + (g/l) = 0
    a = 1.0
    b_coeff = b/(m*l^2)
    c_coeff = g/l
    
    # Discriminante
    discriminant = b_coeff^2 - 4*a*c_coeff
    
    println("Análisis de estabilidad (pequeñas oscilaciones):")
    println("Ecuación característica: s² + $(b_coeff)s + $(c_coeff) = 0")
    println("Discriminante: $(discriminant)")
    
    if discriminant > 0
        s1 = (-b_coeff + sqrt(discriminant))/2
        s2 = (-b_coeff - sqrt(discriminant))/2
        println("Raíces reales: s₁ = $(s1), s₂ = $(s2)")
        println("Sistema sobreamortiguado")
    elseif discriminant == 0
        s = -b_coeff/2
        println("Raíz doble: s = $(s)")
        println("Sistema críticamente amortiguado")
    else
        real_part = -b_coeff/2
        imag_part = sqrt(-discriminant)/2
        println("Raíces complejas: s = $(real_part) ± $(imag_part)i")
        println("Sistema subamortiguado")
        println("Frecuencia natural amortiguada: $(imag_part) rad/s")
    end
end

# Ejecutar ejemplo
#ejemplo_basico()

# Funciones adicionales para guardar gráficas

# Función para guardar una gráfica individual
function save_single_plot(plot_obj, filename::String, format::String="png")
    """
    Guarda una gráfica individual en el formato especificado
    
    Argumentos:
    - plot_obj: objeto de gráfica de Plots.jl
    - filename: nombre del archivo (sin extensión)
    - format: formato de imagen ("png", "pdf", "svg", "eps")
    """
    full_filename = "$(filename).$(format)"
    savefig(plot_obj, full_filename)
    println("Gráfica guardada como: $(full_filename)")
end

# Función para generar y guardar múltiples gráficas con diferentes formatos
function generate_complete_analysis(params::PendulumParams, initial_conditions, time_span; 
                                  save_all=true, base_filename="pendulum_analysis")
    """
    Genera un análisis completo del péndulo y guarda todas las gráficas
    """
    
    # Resolver el sistema
    sol = solve_pendulum(params, initial_conditions, time_span)
    
    # Crear gráficas individuales
    t = sol.t
    θ = [u[1] for u in sol.u]
    θ_dot = [u[2] for u in sol.u]
    
    # Gráfica de ángulo vs tiempo
    p1 = plot(t, θ, label="θ(t)", xlabel="Tiempo (s)", ylabel="Ángulo (rad)", 
              title="Evolución del Ángulo", linewidth=2, size=(600,400))
    
    # Gráfica de velocidad vs tiempo  
    p2 = plot(t, θ_dot, label="θ̇(t)", xlabel="Tiempo (s)", ylabel="Velocidad angular (rad/s)", 
              title="Evolución de la Velocidad Angular", linewidth=2, color=:red, size=(600,400))
    
    # Diagrama de fases
    p3 = plot(θ, θ_dot, label="Trayectoria", xlabel="θ (rad)", ylabel="θ̇ (rad/s)", 
              title="Diagrama de Fases", linewidth=2, color=:green, size=(600,400))
    
    # Energía del sistema
    m, l, g = params.m, params.l, params.g
    kinetic_energy = 0.5 * m * l^2 * θ_dot.^2
    potential_energy = m * g * l * (1 .- cos.(θ))
    total_energy = kinetic_energy + potential_energy
    
    p4 = plot(t, [kinetic_energy potential_energy total_energy], 
              label=["Energía Cinética" "Energía Potencial" "Energía Total"],
              xlabel="Tiempo (s)", ylabel="Energía (J)",
              title="Análisis de Energía", linewidth=2, size=(600,400))
    
    if save_all
        # Guardar en diferentes formatos
        formats = ["png", "pdf", "svg"]
        
        for fmt in formats
            save_single_plot(p1, "$(base_filename)_angle", fmt)
            save_single_plot(p2, "$(base_filename)_velocity", fmt) 
            save_single_plot(p3, "$(base_filename)_phase", fmt)
            save_single_plot(p4, "$(base_filename)_energy", fmt)
        end
        
        # Gráfica combinada
        combined = plot(p1, p2, p3, p4, layout=(2,2), size=(1000,800))
        for fmt in formats
            save_single_plot(combined, "$(base_filename)_combined", fmt)
        end
        
        println("\nTodas las gráficas han sido guardadas en formatos PNG, PDF y SVG")
    end
    
    return (p1, p2, p3, p4)
end

# Ejemplo de uso para guardar gráficas

function ejemplo_con_guardado()
    # Parámetros del péndulo
    params = PendulumParams(1.0, 1.0, 9.81, 0.2, 0.0)
    initial_conditions = [π/4, 0.0]
    time_span = (0.0, 10.0)
    
    # Generar análisis completo y guardar
    generate_complete_analysis(params, initial_conditions, time_span, 
                             save_all=true, base_filename="mi_pendulo")
    
    # También puedes usar las funciones originales con la opción de guardado
    sol = solve_pendulum(params, initial_conditions, time_span)
    plot_pendulum_solution(sol, "Mi Péndulo Personalizado", 
                          save_plots=true, filename="pendulo_personalizado")
    
    # Comparar casos y guardar
    compare_cases(save_comparison=true)
end

function p_simple_boliche()
  """
    # Parámetros del péndulo
    # El sistema es una bola de boliche (m=2.86kg) se empujada por una mano (t=16N*m)
    # la cual esta en constante rosamiento con el viento (amortiguamiento B=0.65).
    # Las estructura de los parametros es la siguiente:
    # -> masa (kg)
    # -> longitud (m)
    # -> gravedad (m/s²)
    # -> coeficiente de amortiguamiento
    # -> torque externo (N⋅m)
  """
    params = PendulumParams(2.85, 1.0, 9.81, 0.65, 16)
    #Condiciones iniciales cuando es solatado en una posicion de 45° con velocidad incial 0
    #initial_conditions = [π/4, 0.0]
    #Condiciones iniciales para cuando bola esta congando (vertical \thetha=0) y velocidad 0
    initial_conditions = [0.0, 0.0]
    #Tiempo para la simulacion 10s
    time_span = (0.0, 50.0)

    # Solicitamos alguna informacion
    print("\n💾 ¿Nombre del archivo?: ")
    filename_init = strip(readline())
    
    print("\n🤔 ¿Quieres realizar un analisis completo? (s/N): ")
    response = strip(readline())
    if lowercase(response) in ["s", "si", "y", "yes"]
        # Generar análisis completo y guardar
        println("🚀 Iniciando análisis del péndulo...")
        generate_complete_analysis(params, initial_conditions, time_span,save_all=true,base_filename=filename_init) 
        println("✅ Analisis completo, verifica las graficas en tu directorio.")
    else
        #Analisamos nuestro caso simple
        # Buscamos la solucion
        println("🚀 Iniciando análisis del péndulo...")
        sol = solve_pendulum(params, initial_conditions, time_span)
        # Graficamos nuestra solucion 
        println("✅ Analisis individual del caso")
        plot_pendulum_solution(sol, "Pendulo (Bola de Boliche)", filename=filename_init)
    
        println("✅ Analisis comparativo del caso")
        # Comparar casos y guardar
        cp_cases = compare_cases(save_comparison=true,filename=filename_init)
        display(cp_cases)
        println("Terminar enter ...")
        readline()
        println("✅ Analisis Finalizado!!")
     end
end

p_simple_boliche()

#Ejemplo para guardado
#ejemplo_con_guardado()
