from django.urls import path, include
from rest_framework.routers import DefaultRouter
from rest_framework_nested import routers

from .views import CanvasViewSet, WorkspaceFileViewSet, execute_command

# Main router for canvases
router = DefaultRouter()
router.register(r'canvases', CanvasViewSet, basename='canvas')

# Nested router for files within a canvas
canvases_router = routers.NestedDefaultRouter(router, r'canvases', lookup='canvas')
canvases_router.register(r'files', WorkspaceFileViewSet, basename='canvas-files')

urlpatterns = [
    path('', include(router.urls)),
    path('', include(canvases_router.urls)),
    path('canvases/<uuid:canvas_id>/execute/', execute_command, name='execute-command'),
]
