from django.db import models
from django.contrib.auth.models import User
import uuid


class Canvas(models.Model):
    """
    Represents a user's workspace/canvas where they build ROS projects
    Each canvas has a unique ID and is stored in Docker at /workspaces/<username>/<canvas_id>/
    """
    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    user = models.ForeignKey(User, on_delete=models.CASCADE, related_name="canvases")
    name = models.CharField(max_length=255)
    description = models.TextField(blank=True)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)
    is_active = models.BooleanField(default=True)

    class Meta:
        verbose_name = "Canvas"
        verbose_name_plural = "Canvases"
        ordering = ["-updated_at"]

    def __str__(self):
        return f"{self.user.username}/{self.name} ({self.id})"

    @property
    def docker_path(self):
        """Returns the Docker path for this canvas"""
        return f"/workspaces/{self.user.username}/{self.id}"


class WorkspaceFile(models.Model):
    """
    Represents a file or directory within a canvas workspace
    Mirrors the file structure in Docker
    """
    FILE = "file"
    DIRECTORY = "directory"
    TYPE_CHOICES = [
        (FILE, "File"),
        (DIRECTORY, "Directory"),
    ]

    id = models.UUIDField(primary_key=True, default=uuid.uuid4, editable=False)
    canvas = models.ForeignKey(Canvas, on_delete=models.CASCADE, related_name="files")
    path = models.CharField(max_length=512, help_text="Relative path from canvas root (e.g., 'src/my_robot/urdf/robot.urdf')")
    file_type = models.CharField(max_length=10, choices=TYPE_CHOICES, default=FILE)
    content = models.TextField(blank=True, help_text="File content (empty for directories)")
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        unique_together = ["canvas", "path"]
        ordering = ["path"]

    def __str__(self):
        return f"{self.canvas.user.username}/{self.canvas.id}/{self.path}"

    @property
    def full_docker_path(self):
        """Returns the full Docker path for this file"""
        return f"{self.canvas.docker_path}/{self.path}"

    @property
    def name(self):
        """Returns the file/directory name"""
        return self.path.split("/")[-1] if "/" in self.path else self.path
