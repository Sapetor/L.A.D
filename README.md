# L.A.D (Learn Autonomous Driving)

L.A.D es un ecosistema pensado para cursos de robótica y conducción autónoma. Reúne tres piezas principales:

1. **Frontend React** (este repositorio) donde el estudiantado consume las unidades, niveles y widgets ROS.
2. **Backend REST** que gestiona autenticación, catálogo de contenidos y progreso individual.
3. **Simuladores ROS 2** (con rosbridge) que habilitan prácticas interactivas directamente desde el navegador.

Este README consolida y amplía toda la documentación previa para los tres repositorios de referencia. Aquí encontrarás desde la visión general hasta guías de despliegue, administración de contenidos y buenas prácticas para operar el laboratorio completo.

---

## Tabla de contenido

1. [Visión general y componentes](#visión-general-y-componentes)
2. [Arquitectura de referencia](#arquitectura-de-referencia)
3. [Requisitos previos](#requisitos-previos)
4. [Variables de entorno](#variables-de-entorno)
5. [Instalación y scripts de npm](#instalación-y-scripts-de-npm)
6. [Estructura del frontend](#estructura-del-frontend)
7. [Backend de referencia](#backend-de-referencia)
8. [Simuladores ROS 2 y rosbridge](#simuladores-ros-2-y-rosbridge)
9. [Orquestación completa con Docker Compose](#orquestación-completa-con-docker-compose)
10. [Administración de unidades, niveles y objetivos](#administración-de-unidades-niveles-y-objetivos)
11. [Personalización del contenido educativo](#personalización-del-contenido-educativo)
12. [Pruebas, QA y monitoreo](#pruebas-qa-y-monitoreo)
13. [Guía de despliegue a producción](#guía-de-despliegue-a-producción)
14. [Solución de problemas frecuentes](#solución-de-problemas-frecuentes)
15. [Buenas prácticas de versionado y colaboración](#buenas-prácticas-de-versionado-y-colaboración)
16. [Roadmap sugerido](#roadmap-sugerido)

---

## Visión general y componentes

L.A.D propone un flujo de aprendizaje basado en **unidades** y **niveles** que mezclan teoría con experimentos ROS. Cada nivel puede incluir slides, videos, evaluaciones y widgets en vivo que se conectan a rosbridge para publicar o suscribirse a tópicos.

### Frontend (este repositorio)

- Construido con **Create React App**.
- Usa **React Router** para la navegación (`/learn/:unitSlug/:levelSlug`).
- Gestiona autenticación y progreso con la **Context API** (`AuthContext`).
- Integra ROS mediante hooks personalizados (`useRoslib`, `useStableRosSub`).
- Almacena el token JWT en `localStorage` y añade cabeceras autenticadas en `apiFetch`.

### Backend (API REST)

- Expone endpoints `POST /api/token/`, `GET /api/units/`, `GET /api/levels/progress/me/`, `POST /api/objectives/complete/`, etc.
- Puede implementarse con Django REST Framework, FastAPI u otro framework que entregue JSON con CORS habilitado.
- Mantiene modelos de Unidades, Niveles, Objetivos, Recursos y Progreso.

### ROS 2 + rosbridge

- Corre `rosbridge_server` sobre el workspace ROS utilizado en clase.
- Expone un WebSocket (`ws://localhost:9090` por defecto).
- Se conecta al frontend por medio de `roslibjs` para interactuar con robots, simuladores y evaluadores automáticos.

---

## Arquitectura de referencia

```
┌──────────────────────┐          ┌────────────────────────────┐          ┌──────────────────────────────┐
│  Frontend React      │  HTTPS   │  API REST / Backend        │   TCP    │  ROS 2 + rosbridge          │
│  (npm start / build) │◀────────▶│  (Django, FastAPI, etc.)   │◀────────▶│  Gazebo, RViz, nodos custom │
└──────────────────────┘          └────────────────────────────┘          └──────────────────────────────┘
         ▲                                      ▲                                        ▲
         │                                      │                                        │
         │ JWT                                  │ DB (PostgreSQL, SQLite)                │ Topics, servicios, acciones
         │                                      │                                        │
```

- **Autenticación**: el frontend solicita un token JWT y lo usa en cada `apiFetch`.
- **Catálogo**: `Learn.jsx` y `UnitPage.jsx` consumen `/api/units/` y fusionan los datos con el progreso devuelto por `/api/levels/progress/me/`.
- **ROS**: los widgets dentro de `src/levels` publican/escuchan tópicos para validar objetivos en tiempo real.

---

## Requisitos previos

| Componente | Versión recomendada | Notas |
| --- | --- | --- |
| Node.js | ≥ 18.x | Requerido para desarrollo del frontend. |
| npm | ≥ 9.x | Incluido con Node. |
| Docker | ≥ 24.x | Facilita el despliegue de backend, base de datos y rosbridge. |
| Docker Compose | v2 | Opcional para orquestar los servicios. |
| Python | ≥ 3.10 | Necesario si implementas el backend de referencia con Django. |
| ROS 2 | Humble / Foxy | Debe incluir `rosbridge_server` y los paquetes de tus misiones. |
| Base de datos | PostgreSQL 14+ (recomendado) | Puedes usar SQLite en desarrollo. |

---

## Variables de entorno

Define las variables en un archivo `.env` en la raíz o inyectadas en el entorno de build.

| Variable | Descripción | Valor por defecto |
| --- | --- | --- |
| `REACT_APP_API_BASE` | URL base del backend REST. | `http://localhost:8000/api` |
| `REACT_APP_ROSBRIDGE_URL` | WebSocket hacia rosbridge. | `ws://localhost:9090` |
| `REACT_APP_ENABLE_SSO` | Activa integraciones externas (ej. Keycloak). | `false` |
| `REACT_APP_DEFAULT_UNIT` | Slug de la unidad mostrada al ingresar. | `null` |

> ⚠️ Las variables prefijadas con `REACT_APP_` se leen en tiempo de build. Cambios posteriores exigen recompilar el frontend o regenerar la imagen Docker.

Para el backend y rosbridge considera además:

- `DJANGO_SECRET_KEY`, `DATABASE_URL`, `CORS_ALLOWED_ORIGINS` (Django).
- `POSTGRES_DB`, `POSTGRES_USER`, `POSTGRES_PASSWORD` (PostgreSQL).
- `ROS_DOMAIN_ID`, `ROS_PACKAGE_PATH` (ROS 2), según tu infraestructura.

---

## Instalación y scripts de npm

```bash
npm install        # Instala dependencias del frontend
npm start          # Servidor de desarrollo http://localhost:3000
npm test           # Ejecuta pruebas de React (Jest + Testing Library)
npm run build      # Compila la versión de producción en build/
npm run lint       # (Opcional) Ejecuta ESLint si está configurado
```

- `npm start` crea un proxy para `/api` si configuras `setupProxy.js`.
- `npm test` corre en modo watch; usa `CI=true npm test` para entornos CI.
- `npm run build` genera archivos estáticos listos para Nginx, S3 o contenedores.

---

## Estructura del frontend

```
src/
├── App.jsx / App.js          # Punto de entrada y rutas
├── components/               # UI reutilizable y widgets ROS
├── context/                  # AuthContext, ProgressContext
├── hooks/                    # useRoslib, useApiFetch, etc.
├── levels/                   # Definiciones de misiones y assets por nivel
├── pages/                    # Home, Learn, UnitPage, LearnLevel
├── parches/                  # Overrides puntuales
├── styles/                   # SCSS modularizado
└── config.js                 # URLs y toggles centrales
```

**Archivos clave**

- `src/context/AuthContext.jsx`: maneja `login`, `logout`, refresh de token y `apiFetch` con cabeceras JWT.
- `src/pages/Learn.jsx`: descarga catálogo, sincroniza progreso y decide la unidad activa.
- `src/pages/LearnLevel.jsx`: renderiza el contenido del nivel y coordina widgets ROS.
- `src/hooks/useRoslib.js`: encapsula la conexión WebSocket con rosbridge, reconexión y limpieza.

---

## Backend de referencia

Aunque puedes implementar la API con cualquier stack, la referencia oficial utiliza **Django + Django REST Framework**.

### Instalación rápida (Django)

```bash
python -m venv venv
source venv/bin/activate
pip install -r requirements.txt
python manage.py migrate
python manage.py createsuperuser
python manage.py runserver 0.0.0.0:8000
```

### Modelos mínimos sugeridos

- `Unit`: `slug`, `title`, `order`, `is_active`.
- `Level`: FK a `Unit`, `slug`, `title`, `order`, `is_active`, `content` (JSON o Markdown), `ros_config`.
- `Objective`: FK a `Level`, `code`, `description`, `points`, `ros_topic`.
- `UserProgress`: FK a `User` y `Level`, `completed`, `objectives_completed` (array).

### Endpoints clave

| Método | Ruta | Descripción |
| --- | --- | --- |
| `POST` | `/api/token/` | Devuelve JWT (`access`, `refresh`). |
| `GET` | `/api/units/` | Lista unidades y niveles asociados. |
| `GET` | `/api/levels/<slug>/` | Recupera detalle completo de un nivel. |
| `GET` | `/api/levels/progress/me/` | Progreso del usuario autenticado. |
| `POST` | `/api/objectives/complete/` | Marca un objetivo como cumplido (se puede integrar con ROS). |

### Administración vía panel

![Panel principal de Django admin mostrando los modelos de Learning](images/dashboard.png)

- Accede a `http://localhost:8000/admin` con un usuario staff.
- Crea unidades, niveles y objetivos siguiendo los formularios del panel.
- Usa `python manage.py loaddata fixtures.json` para cargas masivas.

---

## Simuladores ROS 2 y rosbridge

Los niveles pueden conectarse a cualquier nodo ROS 2 mientras exista rosbridge en el mismo dominio.

### Lanzar rosbridge rápidamente

```bash
docker run --rm -it \
  -p 9090:9090 \
  -v $(pwd)/ros2_ws:/root/ros2_ws \
  osrf/ros:humble-desktop \
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Checklist de configuración

1. **Workspace preparado** con paquetes y launch files para tus misiones.
2. **rosbridge** corriendo en el mismo dominio (configura `ROS_DOMAIN_ID` si usas DDS).
3. **Firewall** habilitando el puerto WebSocket (9090 por defecto).
4. **Simuladores** (Gazebo, Ignition, Webots, RViz) listos para publicar tópicos utilizados en los niveles.

### Integración desde el frontend

- Define los widgets en `src/levels/<unit>/<level>.js` con los tópicos y servicios necesarios.
- Usa los hooks `useRosPublisher`, `useRosSubscriber` y `useRosService` para encapsular la lógica.
- Marca los objetivos cumplidos llamando a la API cuando se recibe la confirmación desde ROS.

---

## Orquestación completa con Docker Compose

```yaml
services:
  db:
    image: postgres:15
    environment:
      - POSTGRES_DB=lad
      - POSTGRES_USER=lad_user
      - POSTGRES_PASSWORD=lad_pass
    volumes:
      - lad_pgdata:/var/lib/postgresql/data

  api:
    image: lad/api:latest
    ports:
      - "8000:8000"
    environment:
      - DJANGO_SECRET_KEY=changeme
      - DATABASE_URL=postgres://lad_user:lad_pass@db:5432/lad
      - CORS_ALLOWED_ORIGINS=http://localhost:3000
    depends_on:
      - db

  rosbridge:
    image: osrf/ros:humble-desktop
    command: ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    ports:
      - "9090:9090"
    volumes:
      - ./ros2_ws:/root/ros2_ws:ro

  frontend:
    build: .
    ports:
      - "3000:3000"
    environment:
      - REACT_APP_API_BASE=http://api:8000/api
      - REACT_APP_ROSBRIDGE_URL=ws://rosbridge:9090
    depends_on:
      - api
      - rosbridge

volumes:
  lad_pgdata:
    driver: local
```

### Persistencia y copias de seguridad

- Usa volúmenes nombrados (`lad_pgdata`) o binds (`./data/db`) para no perder la base de datos.
- Respalda con `docker compose exec db pg_dump -U lad_user lad > backup.sql`.
- Restaura con `cat backup.sql | docker compose exec -T db psql -U lad_user lad`.
- Ejecuta migraciones dentro del contenedor `api` (`docker compose exec api python manage.py migrate`).

---

Si planeas ejecutar **Django**, **PostgreSQL**, **rosbridge** y el **frontend de React** en un mismo servidor y exponer la
aplicación mediante la IP pública o privada de la máquina, sigue estos pasos:

1. **Prepara el host**
   - Actualiza el sistema e instala Docker y Docker Compose (`sudo apt-get update && sudo apt-get install docker.io docker-compose-plugin`).
   - Abre los puertos necesarios en el firewall del servidor: `3000/tcp` (frontend), `8000/tcp` (API) y `9090/tcp` (rosbridge). Si usarás HTTPS tras un proxy, habilita también `80` y `443`.

2. **Configura variables conscientes de la IP**
   - Crea un archivo `.env.server` junto al `docker-compose.yml` con tu IP o dominio:
     ```env
     PUBLIC_HOST=10.0.0.5
     REACT_APP_API_BASE=http://10.0.0.5:8000/api
     REACT_APP_ROSBRIDGE_URL=ws://10.0.0.5:9090
     DJANGO_ALLOWED_HOSTS=10.0.0.5,localhost
     CORS_ALLOWED_ORIGINS=http://10.0.0.5:3000
     ```
   - Sobrescribe las variables del servicio `frontend` con `REACT_APP_API_BASE` y `REACT_APP_ROSBRIDGE_URL` para que el bundle apunte a la IP del servidor.
   - Expone las variables `DJANGO_ALLOWED_HOSTS` y `CORS_ALLOWED_ORIGINS` en el servicio `api` para aceptar peticiones desde tu navegador.

3. **Arranca los servicios**
   - Ejecuta `docker compose --env-file .env.server up -d --build` para compilar el frontend con los valores correctos y levantar los contenedores.
   - Verifica los logs con `docker compose logs -f` y confirma que la API aplica migraciones y que rosbridge quedó escuchando en `0.0.0.0:9090`.

4. **Accede desde otra máquina**
   - En un navegador conectado a la misma red (o vía internet si la IP es pública) visita `http://<tu-ip>:3000`.
   - El frontend consultará `http://<tu-ip>:8000/api` para autenticación y progreso, y abrirá el WebSocket `ws://<tu-ip>:9090` hacia rosbridge.

5. **Endurecimiento opcional**
   - Coloca un proxy inverso (Nginx, Traefik o Caddy) al frente para servir el build de React en HTTPS y redirigir tráfico hacia `/api` y `/ros`.
   - Automatiza la obtención de certificados TLS con Let's Encrypt (`certbot`) o Traefik `acme`.
   - Habilita autenticación multifactor para las cuentas de staff en Django y restringe el acceso al puerto 9090 desde redes de confianza.

## Administración de unidades, niveles y objetivos

1. **Inicia sesión** en `http://localhost:8000/admin` con una cuenta de staff.
2. **Crea unidades** definidas por `slug`, `title`, `order`, `is_active`.
3. **Agrega niveles** a cada unidad y configura su orden de aparición.
4. **Define objetivos** con códigos únicos y puntajes que se corresponden con eventos ROS.
5. **Verifica el catálogo** desde la vista de niveles para confirmar estado y orden.
6. **Prueba en el frontend** con `npm start` para asegurarte de que los cambios se reflejan.

| Pantalla | Captura |
| --- | --- |
| Login del panel | ![Pantalla de inicio de sesión del panel administrativo](images/login.png) |
| Dashboard | ![Panel principal con los modelos de aprendizaje](images/dashboard.png) |
| Crear Unidad | ![Formulario para crear una unidad](images/add-unit.png) |
| Crear Nivel | ![Formulario para crear un nivel con objetivos](images/add-level.png) |
| Crear Objetivo | ![Formulario para crear un objetivo](images/add-objective.png) |

---

## Personalización del contenido educativo

- **Layouts personalizados:** puedes crear nuevos componentes en `src/components` y referenciarlos desde `src/levels`.
- **Assets multimedia:** coloca imágenes o videos en `public/` o en servicios externos (YouTube, Vimeo) y referencia las URLs.
- **Integraciones externas:** activa `REACT_APP_ENABLE_SSO` y agrega el flujo en `AuthContext` para soportar SSO.
- **Idiomas:** utiliza `react-intl` o `i18next` si necesitas internacionalización; mantiene los textos en archivos JSON.
- **Gamificación:** extiende el backend con badges, logros y tablas de clasificación; expón endpoints y consúmelos desde el frontend.

### Checklist para nuevos niveles

1. Define el objetivo pedagógico y los tópicos ROS a utilizar.
2. Prepara el contenido (Markdown, slides, videos) y súbelo a la API.
3. Implementa widgets en `src/levels/<unit>/<level>.js`.
4. Configura el backend con objetivos y validaciones (webhooks o callbacks ROS).
5. Prueba el flujo completo (frontend → backend → ROS → backend → frontend).

---

## Pruebas, QA y monitoreo

- **Frontend:** usa React Testing Library (`npm test`) para validar componentes críticos y hooks personalizados.
- **Backend:** implementa pruebas unitarias y de integración (pytest o Django `TestCase`).
- **ROS:** crea launch tests que verifiquen que los nodos publican/suscriben correctamente.
- **Monitoreo:** integra Prometheus + Grafana o herramientas equivalentes para observar consumo de recursos, latencia y errores.
- **Registro de eventos:** el frontend registra peticiones en consola (`[apiFetch]`); puedes añadir Sentry para telemetría.

---

## Guía de despliegue a producción

1. **Compila el frontend:** `npm run build` y sirve los archivos con Nginx o dentro de un contenedor estático.
2. **Configura HTTPS:** usa un reverse proxy (Nginx, Traefik) con certificados TLS.
3. **Escala el backend:** ejecuta múltiples réplicas detrás de Gunicorn/Uvicorn y un balanceador de carga.
4. **Gestiona secretos:** almacena tokens y contraseñas en Vault, AWS Secrets Manager u opciones similares.
5. **Observabilidad:** habilita logs centralizados (Elastic Stack, Loki) y alertas sobre métricas clave.
6. **Respalda bases de datos y workspaces ROS** antes de cada actualización.

### Deploy con contenedores

- Construye imágenes versionadas (`lad/frontend:1.0.0`, `lad/api:1.0.0`).
- Usa `docker compose -f docker-compose.prod.yml up -d` con configuraciones específicas de producción.
- Considera Kubernetes para clusters multiusuario; define ConfigMaps/Secrets para las variables necesarias.

---

## Solución de problemas frecuentes

| Problema | Posibles causas | Solución |
| --- | --- | --- |
| El frontend no inicia sesión | URL incorrecta en `REACT_APP_API_BASE`, CORS bloqueado | Verifica variables, habilita CORS en el backend, revisa logs de red. |
| Widgets ROS no conectan | rosbridge caído, puerto bloqueado, dominio DDS distinto | Revisa `ros2 topic list`, confirma puerto 9090, alinea `ROS_DOMAIN_ID`. |
| Cambios en `.env` no se reflejan | Build en caché | Ejecuta `npm run build` nuevamente o reconstruye la imagen. |
| Carpetas faltan en GitHub | `.gitignore` las excluye o contienen otro repo | Agrega excepciones (`!carpeta/`) o elimina `.git` internos. |
| `npm start` falla en Windows | Conflictos con rutas o puertos | Corre `set "PORT=3000" && npm start` o usa WSL2. |

---

## Buenas prácticas de versionado y colaboración

1. Revisa `git status` antes de cada commit para confirmar que incluyes `images/`, `ros2_ws/`, etc.
2. Evita repositorios anidados; si necesitas uno, conviértelo en submódulo (`git submodule add`).
3. Usa ramas descriptivas (`feature/nuevo-widget`, `docs/actualizar-manual`).
4. Adjunta capturas o GIFs cuando cambies la UI (carpeta `images/`).
5. Automatiza lint y pruebas en CI (`GitHub Actions`, `GitLab CI`).
6. Documenta breaking changes en el changelog y versiona las imágenes Docker.
7. Recuerda que Git no almacena directorios vacíos: usa `.gitkeep` si necesitas reservarlos.

---

## Roadmap sugerido

- Integrar evaluaciones automáticas conectadas a tópicos ROS para calificar misiones en tiempo real.
- Añadir más escenarios (Turtlesim, Gazebo, Ignition, QCar) y documentarlos dentro de `src/levels`.
- Publicar scripts de bootstrap para preparar ROS 2 + rosbridge en laboratorios universitarios.
- Incorporar analíticas de aprendizaje (tiempo por nivel, objetivos fallidos) en dashboards administrativos.
- Ofrecer soporte multitenant en el backend para múltiples cursos o sedes.

---

