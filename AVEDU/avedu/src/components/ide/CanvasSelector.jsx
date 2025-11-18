import React, { useState, useEffect } from "react";
import { useNavigate } from "react-router-dom";
import * as fileApi from "../../services/fileApi";
import "../../styles/components/_canvas-selector.scss";

/**
 * Canvas Selector - Shows all user canvases as cards with create option
 */
export default function CanvasSelector({ onCanvasSelect }) {
  const navigate = useNavigate();
  const [canvases, setCanvases] = useState([]);
  const [loading, setLoading] = useState(true);
  const [showCreateModal, setShowCreateModal] = useState(false);
  const [newCanvasName, setNewCanvasName] = useState("");
  const [newCanvasDescription, setNewCanvasDescription] = useState("");

  useEffect(() => {
    loadCanvases();
  }, []);

  async function loadCanvases() {
    try {
      setLoading(true);
      const data = await fileApi.listCanvases();
      setCanvases(data);
    } catch (error) {
      console.error("Failed to load canvases:", error);
    } finally {
      setLoading(false);
    }
  }

  async function handleCreateCanvas() {
    if (!newCanvasName.trim()) return;

    try {
      const canvas = await fileApi.createCanvas({
        name: newCanvasName,
        description: newCanvasDescription,
      });
      setCanvases([...canvases, canvas]);
      setShowCreateModal(false);
      setNewCanvasName("");
      setNewCanvasDescription("");
      onCanvasSelect(canvas);
    } catch (error) {
      console.error("Failed to create canvas:", error);
      alert("Failed to create canvas. Please try again.");
    }
  }

  function handleCanvasClick(canvas) {
    onCanvasSelect(canvas);
  }

  if (loading) {
    return (
      <div className="screen">
        <div className="overlay overlay--scan" />
        <div className="overlay overlay--vignette" />
        <div className="stars" aria-hidden />
        <div className="stars stars--2" aria-hidden />
        <div className="stars stars--3" aria-hidden />
        <div className="canvas-selector">
          <div className="canvas-selector__loading">Loading canvases...</div>
        </div>
      </div>
    );
  }

  return (
    <div className="screen">
      <div className="overlay overlay--scan" />
      <div className="overlay overlay--vignette" />
      <div className="stars" aria-hidden />
      <div className="stars stars--2" aria-hidden />
      <div className="stars stars--3" aria-hidden />

      <div className="canvas-selector">
        {/* Home Button */}
        <button className="canvas-selector__home" onClick={() => navigate("/")}>
          ← HOME
        </button>

        <div className="canvas-selector__header">
          <h1 className="title">
            <span>MY WORKSPACES</span>
            <small>SELECT OR CREATE A ROS PROJECT</small>
          </h1>
        </div>

      <div className="canvas-selector__grid">
        {/* Create New Canvas Card - Always first */}
        <div
          className="canvas-card canvas-card--create"
          onClick={() => setShowCreateModal(true)}
        >
          <div className="canvas-card__icon">+</div>
          <div className="canvas-card__title">Create Canvas</div>
          <div className="canvas-card__description">
            Start a new ROS workspace
          </div>
        </div>

        {/* Existing Canvases */}
        {canvases.map((canvas) => (
          <div
            key={canvas.id}
            className="canvas-card"
            onClick={() => handleCanvasClick(canvas)}
          >
            <div className="canvas-card__header">
              <div className="canvas-card__title">{canvas.name}</div>
              <div className="canvas-card__date">
                {new Date(canvas.updated_at).toLocaleDateString()}
              </div>
            </div>
            {canvas.description && (
              <div className="canvas-card__description">
                {canvas.description}
              </div>
            )}
            <div className="canvas-card__meta">
              <span className="canvas-card__id">ID: {canvas.id.slice(0, 8)}...</span>
            </div>
          </div>
        ))}
      </div>

        {/* Create Canvas Modal */}
        {showCreateModal && (
          <div className="canvas-modal" onClick={() => setShowCreateModal(false)}>
            <div className="canvas-modal__content" onClick={(e) => e.stopPropagation()}>
              <div className="canvas-modal__header">
                <h2>Create New Canvas</h2>
                <button
                  className="canvas-modal__close"
                  onClick={() => setShowCreateModal(false)}
                >
                  ×
                </button>
              </div>

              <div className="canvas-modal__body">
                <div className="canvas-modal__field">
                  <label htmlFor="canvas-name">Workspace Name</label>
                  <input
                    id="canvas-name"
                    type="text"
                    placeholder="e.g., My Robot Project"
                    value={newCanvasName}
                    onChange={(e) => setNewCanvasName(e.target.value)}
                    onKeyPress={(e) => e.key === "Enter" && handleCreateCanvas()}
                    autoFocus
                  />
                </div>

                <div className="canvas-modal__field">
                  <label htmlFor="canvas-description">Description (optional)</label>
                  <textarea
                    id="canvas-description"
                    placeholder="Brief description of your project..."
                    value={newCanvasDescription}
                    onChange={(e) => setNewCanvasDescription(e.target.value)}
                    rows={3}
                  />
                </div>
              </div>

              <div className="canvas-modal__footer">
                <button
                  className="canvas-modal__button canvas-modal__button--cancel"
                  onClick={() => setShowCreateModal(false)}
                >
                  Cancel
                </button>
                <button
                  className="canvas-modal__button canvas-modal__button--create"
                  onClick={handleCreateCanvas}
                  disabled={!newCanvasName.trim()}
                >
                  Create Canvas
                </button>
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
