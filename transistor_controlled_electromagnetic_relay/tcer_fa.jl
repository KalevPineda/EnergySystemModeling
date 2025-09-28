
# Importar las bibliotecas necesarias
using DifferentialEquations
using Plots

# --- Parámetros del Sistema (sin cambios) ---
R = 125.0       # Resistencia de la bobina en Ohms
L = 0.075       # Inductancia de la bobina en Henrys
V_CE_sat = 0.2  # Voltaje de saturación del transistor en Volts
V_f = 0.7       # Caída de voltaje del diodo de libre circulación (flyback)

# --- Definición de la Ecuación Diferencial Cíclica ---
# Esta función utiliza el operador módulo para encender y apagar
# el interruptor en un ciclo repetitivo.
# p = (V_in, periodo_on)
function relevador_ciclico(i_c, p, t)
    V_in, periodo_on = p
    periodo_total = 2 * periodo_on
    
    # Determinar la posición en el ciclo actual usando el módulo
    posicion_en_ciclo = t % periodo_total
    
    if posicion_en_ciclo < periodo_on
        # FASE DE ENCENDIDO: El transistor conduce
        return (V_in - V_CE_sat - i_c * R) / L
    else
        # FASE DE APAGADO: La corriente circula por el diodo
        return (-V_f - i_c * R) / L
    end
end

# --- Condiciones de Simulación ---
i_c0 = 0.0          # Condición inicial: corriente cero
periodo_on = 5.0    # 5 segundos en estado ENCENDIDO
num_ciclos = 3      # Número de ciclos completos a simular
t_final = num_ciclos * 2 * periodo_on # Tiempo total de simulación (3 ciclos * 10s/ciclo = 30s)
tspan = (0.0, t_final)

# --- Voltajes de Entrada para Simular ---
voltajes_entrada = [5.0, 9.0, 12.0,24.0,48.0]
resultados = []

# --- Bucle para simular con cada voltaje de entrada ---
for v_in in voltajes_entrada
    # Parámetros para esta simulación específica
    params = (v_in, periodo_on)
    
    # Definir el problema para el ODE solver
    # Usamos saveat para asegurar que tenemos puntos suficientes para una gráfica suave
    problema = ODEProblem(relevador_ciclico, i_c0, tspan, params)
    
    # Resolver la ecuación diferencial
    solucion = solve(problema, Tsit5(), saveat=0.01)
    
    # Guardar los resultados
    push!(resultados, (v_in, solucion))
end

# --- Visualización de los Resultados ---
# Crear un gráfico
p = plot(xlabel="Tiempo (s)", 
         ylabel="Corriente de Bobina (A)", 
         title="Operación Cíclica del Relevador (5s ENC / 5s AP)", 
         legend=:topright)

# Graficar cada una de las soluciones
for res in resultados
    v_in = res[1]
    sol = res[2]
    plot!(p, sol, vars=(0,1), label="V_in = $v_in V")
end

# Añadir líneas verticales para indicar cada conmutación
tiempos_conmutacion = [i * periodo_on for i in 1:(2*num_ciclos - 1)]
vline!(p, tiempos_conmutacion, label="Conmutación", style=:dash, color=:black)

# Mostrar el gráfico
display(p)
savefig(p,"encendido_apagado.png")
