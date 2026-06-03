import streamlit as st
import math
import pandas as pd
import folium
from streamlit_folium import st_folium
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from geopy.geocoders import Nominatim
from geopy.extra.rate_limiter import RateLimiter

# Configuración de página de Streamlit
st.set_page_config(
    page_title="Sabor Sabanero S.A.S. - Optimizador de Rutas",
    page_icon="🚚",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Estilo personalizado (CSS) para dar un look premium, moderno e innovador
st.markdown("""
    <style>
        .main-header {
            font-size: 2.5rem;
            font-weight: 800;
            color: #1E3A8A;
            text-align: center;
            margin-bottom: 0.5rem;
        }
        .subheader {
            font-size: 1.1rem;
            color: #4B5563;
            text-align: center;
            margin-bottom: 2rem;
        }
        .card {
            background-color: #F3F4F6;
            padding: 1.5rem;
            border-radius: 10px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.05);
            margin-bottom: 1rem;
        }
        .metric-title {
            font-size: 0.9rem;
            color: #6B7280;
            font-weight: bold;
        }
        .metric-value {
            font-size: 1.8rem;
            font-weight: bold;
            color: #10B981;
        }
        .stButton>button {
            background-color: #1E3A8A;
            color: white;
            font-weight: bold;
            border-radius: 8px;
            padding: 0.5rem 2rem;
            width: 100%;
            transition: all 0.3s;
        }
        .stButton>button:hover {
            background-color: #3B82F6;
            border-color: #3B82F6;
        }
    </style>
""", unsafe_allow_html=True)

# Inicializar Geocoder con un User-Agent único
geolocator = Nominatim(user_agent="sabor_sabanero_routing_app_v1")
geocode = RateLimiter(geolocator.geocode, min_delay_seconds=1)

# Inicialización del estado de sesión para guardar los puntos dinámicamente
if 'puntos' not in st.session_state:
    # Datos por defecto basados en tu archivo original
    st.session_state.puntos = [
        {"nombre": "CEDI Tocancipá", "lat": 4.964, "lon": -73.912, "demanda": 0, "es_cedi": True},
        {"nombre": "Chía", "lat": 4.863, "lon": -74.053, "demanda": 1100, "es_cedi": False},
        {"nombre": "Cajicá", "lat": 4.918, "lon": -74.029, "demanda": 750, "es_cedi": False},
        {"nombre": "Zipaquirá", "lat": 4.996, "lon": -74.003, "demanda": 1400, "es_cedi": False},
        {"nombre": "Sopó", "lat": 4.908, "lon": -73.938, "demanda": 900, "es_cedi": False},
        {"nombre": "Briceño", "lat": 4.945, "lon": -73.921, "demanda": 500, "es_cedi": False}
    ]

# ==========================================
# CÁLCULOS GEOGRÁFICOS Y DE RUTA
# ==========================================
def calcular_distancia_euclidiana(coord1, coord2):
    """Calcula la distancia usando el factor de 111,000 metros/grado"""
    lat1, lon1 = coord1[0], coord1[1]
    lat2, lon2 = coord2[0], coord2[1]
    distancia_grados = math.sqrt((lat2 - lat1)**2 + (lon2 - lon1)**2)
    return int(distancia_grados * 111000)

def resolver_ruteo(puntos, num_vehiculos, capacidad_vehiculo):
    # Estructurar modelo de datos
    coordenadas = [[p['lat'], p['lon']] for p in puntos]
    nombres = [p['nombre'] for p in puntos]
    demandas = [p['demanda'] for p in puntos]
    
    num_puntos = len(coordenadas)
    matriz_distancias = []
    for i in range(num_puntos):
        fila = []
        for j in range(num_puntos):
            fila.append(calcular_distancia_euclidiana(coordenadas[i], coordenadas[j]))
        matriz_distancias.append(fila)

    datos = {
        'coordenadas': coordenadas,
        'nombres_nodos': nombres,
        'matriz_distancias': matriz_distancias,
        'demandas': demandas,
        'capacidades_vehiculos': [capacidad_vehiculo] * num_vehiculos,
        'num_vehiculos': num_vehiculos,
        'deposito': 0  # El primer elemento siempre es el CEDI
    }

    # Configuración del optimizador OR-Tools
    manager = pywrapcp.RoutingIndexManager(len(datos['matriz_distancias']), datos['num_vehiculos'], datos['deposito'])
    routing = pywrapcp.RoutingModel(manager)

    def callback_distancia(desde_index, hacia_index):
        desde_nodo = manager.IndexToNode(desde_index)
        hacia_nodo = manager.IndexToNode(hacia_index)
        return datos['matriz_distancias'][desde_nodo][hacia_nodo]

    transit_callback_index = routing.RegisterTransitCallback(callback_distancia)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    def callback_demanda(desde_index):
        desde_nodo = manager.IndexToNode(desde_index)
        return datos['demandas'][desde_nodo]

    demand_callback_index = routing.RegisterUnaryTransitCallback(callback_demanda)

    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,
        datos['capacidades_vehiculos'],
        True,
        'Capacidad'
    )

    parametros_busqueda = pywrapcp.DefaultRoutingSearchParameters()
    parametros_busqueda.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    parametros_busqueda.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    parametros_busqueda.time_limit.seconds = 2

    solucion = routing.SolveWithParameters(parametros_busqueda)

    if solucion:
        return extraer_rutas(datos, manager, routing, solucion)
    else:
        return None

def extraer_rutas(data, manager, routing, solution):
    rutas = []
    for id_vehiculo in range(data['num_vehiculos']):
        ruta_vehiculo = []
        index = routing.Start(id_vehiculo)
        distancia_acumulada = 0

        while not routing.IsEnd(index):
            nodo_actual = manager.IndexToNode(index)
            indice_anterior = index
            index = solution.Value(routing.NextVar(index))
            distancia_tramo = routing.GetArcCostForVehicle(indice_anterior, index, id_vehiculo)
            distancia_acumulada += distancia_tramo

            ruta_vehiculo.append({
                'nodo': nodo_actual,
                'nombre': data['nombres_nodos'][nodo_actual],
                'coordenadas': data['coordenadas'][nodo_actual],
                'demanda': data['demandas'][nodo_actual]
            })

        nodo_final = manager.IndexToNode(index)
        ruta_vehiculo.append({
            'nodo': nodo_final,
            'nombre': data['nombres_nodos'][nodo_final],
            'coordenadas': data['coordenadas'][nodo_final],
            'demanda': data['demandas'][nodo_final]
        })

        rutas.append({
            'id_vehiculo': id_vehiculo + 1,
            'trayecto': ruta_vehiculo,
            'distancia_total_m': distancia_acumulada,
            'capacidad_maxima': data['capacidades_vehiculos'][id_vehiculo]
        })
    return rutas

# ==========================================
# ENTORNO VISUAL Y CONTROLES (STREAMLIT)
# ==========================================

# Título de la Aplicación
st.markdown("<div class='main-header'>🚚 Sabor Sabanero S.A.S.</div>", unsafe_allow_html=True)
st.markdown("<div class='subheader'>Consola de Optimización Inteligente de Ruteo de Vehículos (VRP)</div>", unsafe_allow_html=True)

# Sidebar - Gestión de Vehículos y Configuración Global
st.sidebar.image("https://img.icons8.com/isometric/512/truck.png", width=80)
st.sidebar.header("⚙️ Configuración de Flota")
num_vehiculos = st.sidebar.number_input("Número de furgones activos", min_value=1, max_value=10, value=3, step=1)
capacidad_furgon = st.sidebar.number_input("Capacidad de cada furgón (Kg)", min_value=500, max_value=10000, value=2200, step=100)

# Sidebar - Buscador Geográfico (Geocodificación)
st.sidebar.markdown("---")
st.sidebar.header("🔍 Buscar y Añadir Destino")
search_query = st.sidebar.text_input("Buscar dirección (Ej: Chia, Cundinamarca o Sopó)", "")

if st.sidebar.button("Buscar Lugar"):
    if search_query:
        try:
            with st.spinner("Buscando en mapa..."):
                location = geocode(search_query)
                if location:
                    st.sidebar.success(f"📍 Encontrado: {location.address[:40]}...")
                    # Añadir pre-poblado a las cajas de entrada de abajo
                    st.session_state['new_lat'] = location.latitude
                    st.session_state['new_lon'] = location.longitude
                    st.session_state['new_name'] = search_query.split(',')[0]
                else:
                    st.sidebar.error("No se pudo encontrar la localización. Prueba ingresando 'Municipio, Cundinamarca, Colombia'")
        except Exception as e:
            st.sidebar.error("Error en la geolocalización de OpenStreetMap.")

# Formulario para añadir nuevos puntos manualmente o mediante la búsqueda
st.sidebar.markdown("---")
st.sidebar.subheader("➕ Agregar Punto Manualmente")
new_name = st.sidebar.text_input("Nombre de la ubicación", value=st.session_state.get('new_name', ""))
new_lat = st.sidebar.number_input("Latitud", format="%.6f", value=st.session_state.get('new_lat', 4.9))
new_lon = st.sidebar.number_input("Longitud", format="%.6f", value=st.session_state.get('new_lon', -74.0))
new_demand = st.sidebar.number_input("Carga requerida (Kg)", min_value=0, max_value=5000, value=500)

if st.sidebar.button("➕ Insertar a la lista"):
    if new_name:
        st.session_state.puntos.append({
            "nombre": new_name,
            "lat": new_lat,
            "lon": new_lon,
            "demanda": new_demand,
            "es_cedi": False
        })
        st.sidebar.success(f"Añadido {new_name} con éxito.")
        # Limpiar campos de búsqueda auxiliares
        if 'new_name' in st.session_state: del st.session_state['new_name']
        if 'new_lat' in st.session_state: del st.session_state['new_lat']
        if 'new_lon' in st.session_state: del st.session_state['new_lon']
        st.rerun()
    else:
        st.sidebar.warning("Por favor asigne un nombre al lugar")

# Resetear la app
if st.sidebar.button("🗑️ Restablecer Puntos por Defecto"):
    if 'puntos' in st.session_state:
        del st.session_state.puntos
    st.rerun()

# ==========================================
# CONTENIDO PRINCIPAL: GESTIÓN DE PUNTOS
# ==========================================
col_tabla, col_mapa = st.columns([2, 3])

with col_tabla:
    st.subheader("📍 Lista de Destinos y Demandas")
    st.write("El primer nodo es asignado automáticamente como el **Centro de Distribución (CEDI)**.")
    
    # Renderizar tabla editable o controlable
    puntos_df = pd.DataFrame(st.session_state.puntos)
    
    # Permitir borrar puntos individualmente
    for idx, row in puntos_df.iterrows():
        c1, c2, c3, c4 = st.columns([3, 2, 2, 1])
        with c1:
            st.markdown(f"**{row['nombre']}**" + (" 🏪 *(CEDI)*" if row['es_cedi'] else ""))
        with c2:
            st.text(f"{row['lat']:.4f}, {row['lon']:.4f}")
        with c3:
            if not row['es_cedi']:
                # Input para cambiar la demanda directamente
                nueva_dem = st.number_input(f"Kg (Ind. {idx})", min_value=10, max_value=5000, value=int(row['demanda']), step=50, key=f"dem_{idx}")
                st.session_state.puntos[idx]['demanda'] = nueva_dem
            else:
                st.text("Carga: 0 Kg")
        with c4:
            if not row['es_cedi']:
                if st.button("🗑️", key=f"del_{idx}"):
                    st.session_state.puntos.pop(idx)
                    st.rerun()

# ==========================================
# EJECUTAR EL RUTEADOR Y MOSTRAR RESULTADOS
# ==========================================
st.markdown("---")
btn_calcular = st.button("🚀 EJECUTAR OPTIMIZACIÓN DE RUTAS")

# Ejecutar Algoritmo por defecto al cargar o al presionar el botón
rutas_finales = resolver_ruteo(st.session_state.puntos, num_vehiculos, capacidad_furgon)

if rutas_finales:
    # 1. Indicadores Métricas Globales
    distancia_total_m = sum(r['distancia_total_m'] for r in rutas_finales)
    carga_total_kg = sum(sum(p['demanda'] for p in r['trayecto']) for r in rutas_finales)
    furgones_activos = sum(1 for r in rutas_finales if sum(p['demanda'] for p in r['trayecto']) > 0)
    
    st.subheader("📊 Consolidado de la Operación")
    m1, m2, m3 = st.columns(3)
    with m1:
        st.markdown(f"<div class='card'><span class='metric-title'>🏁 DISTANCIA TOTAL COMBINADA</span><br><span class='metric-value'>{(distancia_total_m / 1000):.2f} Km</span></div>", unsafe_allow_html=True)
    with m2:
        st.markdown(f"<div class='card'><span class='metric-title'>📦 TOTAL MERCANCÍA DESPACHADA</span><br><span class='metric-value'>{carga_total_kg:,} Kg</span></div>", unsafe_allowed_html=True)
    with m3:
        st.markdown(f"<div class='card'><span class='metric-title'>🚚 FLOTA EN MOVIMIENTO</span><br><span class='metric-value'>{furgones_activos} / {num_vehiculos} Furgones</span></div>", unsafe_allow_html=True)

    # 2. Visualización Dual: Detalle + Mapa en Dos Columnas
    col_det, col_map = st.columns([2, 3])
    
    with col_det:
        st.subheader("📋 Plan de Ruta para Conductores")
        for r in rutas_finales:
            carga_vehiculo = sum(p['demanda'] for p in r['trayecto'])
            eficiencia = (carga_vehiculo / r['capacidad_maxima']) * 100
            
            # Solo mostrar furgones que tienen asignación
            if carga_vehiculo > 0:
                with st.expander(f"🟢 Furgón #{r['id_vehiculo']} ({carga_vehiculo} kg asignados)", expanded=True):
                    st.markdown(f"**Rendimiento:** {eficiencia:.1f}% de capacidad de bodega usada.")
                    st.markdown(f"**Recorrido estimado:** {(r['distancia_total_m']/1000):.2f} Kilómetros")
                    st.markdown("**Secuencia de Paradas:**")
                    
                    secuencia = []
                    for paso, punto in enumerate(r['trayecto']):
                        detalle_carga = f"(+ {punto['demanda']} Kg)" if punto['demanda'] > 0 else ""
                        secuencia.append(f"📍 **{punto['nombre']}** {detalle_carga}")
                    st.write(" ➔ ".join(secuencia))
            else:
                st.markdown(f"⚪ *Furgón #{r['id_vehiculo']} está libre y en reserva (No requerido).*")

    with col_map:
        st.subheader("🗺️ Mapa de Rutas Interactivo (Bogotá y Sabana)")
        
        # Centrar el mapa alrededor del CEDI (primer punto de la lista)
        cedi_lat = st.session_state.puntos[0]['lat']
        cedi_lon = st.session_state.puntos[0]['lon']
        
        m = folium.Map(location=[cedi_lat, cedi_lon], zoom_start=11, tiles="OpenStreetMap")
        
        colores_rutas = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2']
        
        # 1. Dibujar los puntos del mapa
        for idx, p in enumerate(st.session_state.puntos):
            if p['es_cedi']:
                folium.Marker(
                    location=[p['lat'], p['lon']],
                    popup=f"🏢 {p['nombre']} (CEDI Principal)",
                    tooltip="CEDI de Operación",
                    icon=folium.Icon(color='red', icon='cloud')
                ).add_to(m)
            else:
                folium.Marker(
                    location=[p['lat'], p['lon']],
                    popup=f"📦 {p['nombre']} - Demanda: {p['demanda']} Kg",
                    tooltip=p['nombre'],
                    icon=folium.Icon(color='blue', icon='shopping-cart')
                ).add_to(m)
        
        # 2. Dibujar las líneas de ruteo
        for r in rutas_finales:
            carga_v = sum(p['demanda'] for p in r['trayecto'])
            if carga_v > 0:
                color = colores_rutas[(r['id_vehiculo'] - 1) % len(colores_rutas)]
                coordenadas_trayecto = [[p['coordenadas'][0], p['coordenadas'][1]] for p in r['trayecto']]
                
                # Crear trazo de línea en el mapa
                folium.PolyLine(
                    locations=coordenadas_trayecto,
                    color=color,
                    weight=4,
                    opacity=0.8,
                    tooltip=f"Ruta Furgón {r['id_vehiculo']}"
                ).add_to(m)
                
                # Flechas o dirección (Hacer un marcador sutil a mitad del camino)
                for i in range(len(coordenadas_trayecto) - 1):
                    p1 = coordenadas_trayecto[i]
                    p2 = coordenadas_trayecto[i+1]
                    # Mitad de camino para marcar el sentido de la ruta
                    mid_lat = (p1[0] + p2[0]) / 2
                    mid_lon = (p1[1] + p2[1]) / 2
                    folium.CircleMarker(
                        location=[mid_lat, mid_lon],
                        radius=3,
                        color=color,
                        fill=True,
                        tooltip=f"Furgón {r['id_vehiculo']} en tránsito hacia {r['trayecto'][i+1]['nombre']}"
                    ).add_to(m)

        # Mostrar mapa en Streamlit
        st_folium(m, width="100%", height=450, returned_objects=[])

else:
    st.error("❌ No se encontró una combinación de ruta óptima que cumpla con las capacidades de la flota. Agrega más vehículos o reduce la carga.")
