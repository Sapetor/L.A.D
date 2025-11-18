from rest_framework import serializers
from .models import Canvas, WorkspaceFile


class WorkspaceFileSerializer(serializers.ModelSerializer):
    """Serializer for workspace files"""
    name = serializers.ReadOnlyField()
    full_docker_path = serializers.ReadOnlyField()
    
    class Meta:
        model = WorkspaceFile
        fields = [
            "id",
            "path",
            "name",
            "file_type",
            "content",
            "full_docker_path",
            "created_at",
            "updated_at",
        ]
        read_only_fields = ["id", "created_at", "updated_at"]


class CanvasSerializer(serializers.ModelSerializer):
    """Serializer for canvas/workspace"""
    docker_path = serializers.ReadOnlyField()
    files = WorkspaceFileSerializer(many=True, read_only=True)
    file_count = serializers.SerializerMethodField()
    
    class Meta:
        model = Canvas
        fields = [
            "id",
            "name",
            "description",
            "docker_path",
            "files",
            "file_count",
            "is_active",
            "created_at",
            "updated_at",
        ]
        read_only_fields = ["id", "created_at", "updated_at"]
    
    def get_file_count(self, obj):
        return obj.files.count()


class FileTreeSerializer(serializers.Serializer):
    """Serializer for file tree structure"""
    name = serializers.CharField()
    path = serializers.CharField()
    type = serializers.ChoiceField(choices=["file", "directory"])
    children = serializers.ListField(required=False, allow_null=True)
    unsaved = serializers.BooleanField(required=False, default=False)
