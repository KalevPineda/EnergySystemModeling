
# Importar las bibliotecas necesarias
using DifferentialEquations
using Plots

#Configurar el backed grafico 
try 
  gr()
  println("✅ Backend GR configurado")
catch e
  println("❌ Error configurando GR: $e")
end


# --- Parámetros del Sistema (Relevador Omron G5LA-14 y Transistor 2N2222) ---
R = 125.0       # Resistencia de la bobina en Ohms
L = 0.075       # Inductancia de la bobina en Henrys
V_CE_sat = 0.2  # Voltaje de saturación del transistor en Volts

# --- Definición de la Ecuación Diferencial ---
# La función define la dinámica del sistema d(i_c)/dt
# p es un vector de parámetros, en este caso [V_in]
function relevador_modelo(i_c, p, t)
    V_in = p[1]
    return (V_in - V_CE_sat - i_c * R) / L
end

# --- Condiciones de Simulación ---
i_c0 = 0.0              # Condición inicial: la corriente es cero al inicio
tspan = (0.0, 0.5)     # Intervalo de tiempo para la simulación (0 a 50 ms)

# --- Voltajes de Entrada para Simular ---
voltajes_entrada = [5.0, 9.0, 12.0,24.0,48.0]
resultados = []

# --- Bucle para simular con cada voltaje de entrada ---
for v_in in voltajes_entrada
    # Definir el problema para el ODE solver
    problema = ODEProblem(relevador_modelo, i_c0, tspan, [v_in])
    
    # Resolver la ecuación diferencial
    solucion = solve(problema)
    
    # Guardar los resultados
    push!(resultados, (v_in, solucion))
end

# --- Visualización de los Resultados ---
# Crear un gráfico
p = plot(xlabel="Tiempo (s)", ylabel="Corriente de Colector (A)", title="Respuesta del Relevador a Diferentes Voltajes", legend=:bottomright)

# Graficar cada una de las soluciones
for res in resultados
    v_in = res[1]
    sol = res[2]
    plot!(p, sol, vars=(0,1), label="V_in = $v_in V")
end

# Mostrar el gráfico
display(p)
savefig(p,"tcer.png")
# --- Análisis Adicional: Corriente en Estado Estacionario ---
println("Análisis de la Corriente en Estado Estacionario:")
for v_in in voltajes_entrada
    corriente_ss = (v_in - V_CE_sat) / R
    println("Para V_in = $v_in V, la corriente de estado estacionario es ≈ $(round(corriente_ss * 1000, digits=2)) mA")
end


