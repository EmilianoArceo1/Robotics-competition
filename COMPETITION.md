# Modo Indoor Exploration Competition

Este proyecto contiene un núcleo específico para probar policies contra el
contrato de `indoor-exploration-competition`. No usa la física continua del
modo genérico.

## Uso

1. Inicia `main.py`.
2. Pulsa **CARGAR ARCHIVO .SIM** y elige `Assets/competition_maps/envN/envN.sim`.
3. Opcionalmente pulsa **CARGAR POLICY .PY** y selecciona un archivo que defina
   `class Policy(BasePolicy)`. `Policies/competition_nearest_frontier.py` es un
   ejemplo directamente editable.
4. Inicia la simulación. La capa azul muestra únicamente el conocimiento que
   corresponde a la vista elegida. El HUD siempre muestra la cobertura
   puntuable que sí llegó a la estación base.

La sección **VISTA DE COMPETICIÓN** permite alternar entre:

- **Exploración en vivo**: unión de lo conocido por todos los robots. Es la
  mejor vista para seguir visualmente el mapeo.
- **Reportado a base**: reproduce exactamente el mapa usado para el score; es
  normal que no cambie mientras ningún robot tenga comunicación o complete un
  relay.
- **Robot 1**: observación privada y conocimiento recibido por ese robot.

La escala gráfica de los drones también se ajusta allí. No modifica dinámica,
colisiones ni score.

## Contrato reproducido

- Mapas oficiales reducidos por mínimo en bloques 2x2 y padding de 200 píxeles.
- Coordenadas `(row, col)`, etiquetas libre/desconocido/ocupado `0/0.5/1`.
- LiDAR 15 m, 2500 rayos y 10 píxeles por metro.
- Movimiento A* cardinal con avance máximo de tres celdas por timestep.
- Inicio compartido y delays `0, 5, 10, ...`.
- Mapas privados, fusión robot-robot, trayectorias e intents potencialmente
  obsoletos y reporte a estación base.
- Comunicación por distancia, pérdida logarítmica y atenuación por paredes.
- Modos `explore`, `relay`, `final_relay`, relevo periódico, retorno final y
  transferencia de relay.
- Validación estricta de goals y score de cobertura reportada a base.

Los módulos raíz `base_policy.py`, `policy_utils.py` y `pyastar2d.py` forman la
capa de compatibilidad para que una entrega pueda usar los mismos imports que
en el repositorio oficial.

## Tuning y tablas experimentales

El panel permite ajustar `wIG`, `wC`, `wR` y `wL`. La suma debe ser exactamente
1.00. Sin una policy externa cargada, esos pesos activan `WeightedUtilityPolicy`,
que normaliza information gain, travel cost, redundancia y riesgo de relay.
El cálculo de information gain implementa la interfaz abstracta
`InformationGainMethod`; desde el panel se puede elegir conteo circular de
desconocidos o densidad de frontera y ajustar su radio.

### Next-Best View

La policy integrada **Next Best View** separa regiones a explorar de posiciones
desde las que observarlas. Genera candidatos known-free en centros y bordes de
frontera, offsets alrededor de ellos y celdas cercanas a paredes que pueden
actuar como viewpoints de puertas o esquinas. Luego elimina implícitamente los
inalcanzables mediante A* y evalúa:

```text
U(v) = wIG·IG(v) - wC·cost(v) - wR·redundancy(v) - wL·relayRisk(v)
```

`PotentialVisibilityInformationGain` proyecta rayos solamente sobre el mapa
observado del robot: una pared conocida detiene el rayo y las celdas unknown se
cuentan como potencial, nunca como ground truth. La submission editable está en
`Policies/competition_next_best_view.py`.

**EXPORTAR TABLA EXPERIMENTAL** guarda el resumen de las ejecuciones terminadas
y un CSV hermano con los trials crudos. Para un sweep reproducible completo:

```powershell
python run_competition_sweep.py --step 0.25 --starts 15 15 --starts 40 20
```

El directorio de salida contiene:

- `raw.csv`: una fila por pesos, mapa, robots y pose inicial.
- `summary.csv`: Small, Medium, Large, Single, Multi, Mean, Median, Worst y StdDev.
- `leave_one_out.csv`: selección en seis entornos y validación en el séptimo.

Se usan automáticamente los budgets oficiales según tamaño y track. Para una
prueba rápida de infraestructura puede reducirse temporalmente el número de
rayos con `--num-laser`; los resultados comparables deben usar el valor oficial
de 2500.
