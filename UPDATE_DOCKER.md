# Actualizar Docker con correcciones CORS

Se han realizado los siguientes cambios para corregir el error de CORS:

## Cambios realizados:

1. **entrypoint.sh**: Ahora genera automáticamente `CORS_ALLOW_ORIGIN` incluyendo localhost, 127.0.0.1 y la IP detectada
2. **web_viz.launch.py**: El servidor HTTP ahora recibe explícitamente la variable de entorno CORS
3. **docker-compose.yml**: Usa `${EXPOSED_IP}` del archivo `.env` generado por el script de inicio

## Para aplicar los cambios:

### Opción 1: Reconstruir solo si es necesario
```bash
cd qcar_docker
docker compose up -d --build
```

### Opción 2: Forzar reconstrucción completa
```bash
cd qcar_docker
docker compose down
docker compose build --no-cache
docker compose up -d
```

### Opción 3: Usar el script de inicio (recomendado)
```bash
# Windows
scripts\start-all.bat

# Linux/Mac
./scripts/start-all.sh
```

## Verificación:

Después de reconstruir, verifica que CORS esté funcionando:

1. Abre la consola del navegador (F12)
2. Deberías ver el URDF cargándose sin errores CORS
3. El servidor HTTP en puerto 7000 debería mostrar en sus logs:
   ```
   Serving with CORS on :7000 (CORS_ALLOW_ORIGIN=http://localhost:3000,http://127.0.0.1:3000,http://TU_IP:3000)
   ```

## Logs útiles:

```bash
# Ver logs del contenedor
docker compose -f qcar_docker/docker-compose.yml logs ros

# Ver solo las últimas líneas
docker compose -f qcar_docker/docker-compose.yml logs --tail=50 ros
```
