# Solución: Turtlesim no funciona

## Problema identificado
El error era: `Service /spawn does not exist`

Esto se debía a que turtlesim no se estaba ejecutando correctamente debido a un problema con el display X11.

## Causa raíz
El comando usaba `xvfb-run -s "-screen 0 800x600x24"` que estaba fallando porque intentaba crear un nuevo servidor Xvfb cuando ya había uno corriendo en `:99` para Gazebo.

## Solución aplicada
Se modificó el launch file para usar el mismo display `:99` que Gazebo:

**Antes:**
```python
turtlesim = ExecuteProcess(
    cmd=['bash', '-lc', 'xvfb-run -s "-screen 0 800x600x24" ros2 run turtlesim turtlesim_node'],
    output='screen', condition=IfCondition(enable_turtle)
)
```

**Después:**
```python
turtlesim = ExecuteProcess(
    cmd=['ros2', 'run', 'turtlesim', 'turtlesim_node'],
    output='screen',
    condition=IfCondition(enable_turtle),
    additional_env={'DISPLAY': ':99'}  # Use the same Xvfb display as Gazebo
)
```

## Para aplicar la corrección:

```bash
cd qcar_docker
docker compose down
docker compose build --no-cache
docker compose up -d
```

Espera 60-90 segundos para que Gazebo y turtlesim inicialicen.

## Verificación:

Después de reiniciar, verifica que turtlesim esté corriendo:

```bash
# Ver logs
docker compose -f qcar_docker/docker-compose.yml logs ros | grep turtlesim

# Verificar servicios de turtlesim
docker compose -f qcar_docker/docker-compose.yml exec ros bash -c "source /opt/ros/humble/setup.bash && ros2 service list | grep turtle"
```

Deberías ver servicios como:
- `/clear`
- `/kill`
- `/reset`
- `/spawn`
- `/turtle1/set_pen`
- `/turtle1/teleport_absolute`
- `/turtle1/teleport_relative`

¡Turtlesim ahora debería funcionar correctamente! 🐢
