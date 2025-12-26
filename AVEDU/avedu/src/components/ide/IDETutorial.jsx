// components/ide/IDETutorial.jsx
// Interactive tutorial overlay for IDE with step-by-step guidance

import React, { useState, useEffect, useCallback } from "react";
import PropTypes from "prop-types";
import "./IDETutorial.scss";

/**
 * Tutorial step component - highlights elements and shows instructions
 */
export function IDETutorial({ steps = [], onComplete, onSkip, autoStart = false }) {
  const [currentStep, setCurrentStep] = useState(-1); // -1 means not started
  const [isActive, setIsActive] = useState(false);
  const [highlightedElement, setHighlightedElement] = useState(null);

  // Start tutorial
  const startTutorial = useCallback(() => {
    setCurrentStep(0);
    setIsActive(true);
  }, []);

  // Auto-start if enabled
  useEffect(() => {
    if (autoStart && steps.length > 0) {
      startTutorial();
    }
  }, [autoStart, steps.length, startTutorial]);

  // Update highlighted element when step changes
  useEffect(() => {
    if (currentStep >= 0 && currentStep < steps.length) {
      const step = steps[currentStep];

      if (step.target) {
        // Find target element
        const element = document.querySelector(step.target);
        setHighlightedElement(element);

        // Scroll into view if needed
        if (element && step.scrollIntoView) {
          element.scrollIntoView({ behavior: "smooth", block: "center" });
        }
      } else {
        setHighlightedElement(null);
      }
    } else {
      setHighlightedElement(null);
    }
  }, [currentStep, steps]);

  // Navigation handlers
  const nextStep = useCallback(() => {
    if (currentStep < steps.length - 1) {
      setCurrentStep(currentStep + 1);
    } else {
      // Tutorial completed
      setIsActive(false);
      setCurrentStep(-1);
      onComplete?.();
    }
  }, [currentStep, steps.length, onComplete]);

  const prevStep = useCallback(() => {
    if (currentStep > 0) {
      setCurrentStep(currentStep - 1);
    }
  }, [currentStep]);

  const skipTutorial = useCallback(() => {
    setIsActive(false);
    setCurrentStep(-1);
    onSkip?.();
  }, [onSkip]);

  // Keyboard navigation
  useEffect(() => {
    if (!isActive) return;

    const handleKeyDown = (e) => {
      if (e.key === "ArrowRight" || e.key === "Enter") {
        e.preventDefault();
        nextStep();
      } else if (e.key === "ArrowLeft") {
        e.preventDefault();
        prevStep();
      } else if (e.key === "Escape") {
        e.preventDefault();
        skipTutorial();
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    return () => window.removeEventListener("keydown", handleKeyDown);
  }, [isActive, nextStep, prevStep, skipTutorial]);

  // Don't render if no steps or not active
  if (!isActive || currentStep < 0 || currentStep >= steps.length) {
    return null;
  }

  const step = steps[currentStep];
  const progress = ((currentStep + 1) / steps.length) * 100;

  // Calculate position for tooltip
  let tooltipStyle = {};
  if (highlightedElement) {
    const rect = highlightedElement.getBoundingClientRect();
    const position = step.position || "bottom";
    const offset = step.offset || 20;

    switch (position) {
      case "top":
        tooltipStyle = {
          top: `${rect.top - offset}px`,
          left: `${rect.left + rect.width / 2}px`,
          transform: "translate(-50%, -100%)",
        };
        break;
      case "bottom":
        tooltipStyle = {
          top: `${rect.bottom + offset}px`,
          left: `${rect.left + rect.width / 2}px`,
          transform: "translate(-50%, 0)",
        };
        break;
      case "left":
        tooltipStyle = {
          top: `${rect.top + rect.height / 2}px`,
          left: `${rect.left - offset}px`,
          transform: "translate(-100%, -50%)",
        };
        break;
      case "right":
        tooltipStyle = {
          top: `${rect.top + rect.height / 2}px`,
          left: `${rect.right + offset}px`,
          transform: "translate(0, -50%)",
        };
        break;
      default:
        tooltipStyle = {
          top: "50%",
          left: "50%",
          transform: "translate(-50%, -50%)",
        };
    }
  } else {
    // Center if no target
    tooltipStyle = {
      top: "50%",
      left: "50%",
      transform: "translate(-50%, -50%)",
    };
  }

  return (
    <>
      {/* Backdrop */}
      <div className="ide-tutorial__backdrop" onClick={step.backdropClickable ? nextStep : undefined} />

      {/* Highlight spotlight */}
      {highlightedElement && (
        <div
          className="ide-tutorial__highlight"
          style={{
            top: highlightedElement.getBoundingClientRect().top - 4,
            left: highlightedElement.getBoundingClientRect().left - 4,
            width: highlightedElement.getBoundingClientRect().width + 8,
            height: highlightedElement.getBoundingClientRect().height + 8,
          }}
        />
      )}

      {/* Tutorial tooltip */}
      <div className="ide-tutorial__tooltip" style={tooltipStyle}>
        {/* Progress bar */}
        <div className="ide-tutorial__progress">
          <div className="ide-tutorial__progress-bar" style={{ width: `${progress}%` }} />
        </div>

        {/* Content */}
        <div className="ide-tutorial__content">
          <div className="ide-tutorial__header">
            <h3 className="ide-tutorial__title">{step.title}</h3>
            <div className="ide-tutorial__step-count">
              {currentStep + 1} / {steps.length}
            </div>
          </div>

          <p className="ide-tutorial__text">{step.text}</p>

          {step.warning && (
            <div className="ide-tutorial__warning">
              ⚠️ {step.warning}
            </div>
          )}

          {step.tip && (
            <div className="ide-tutorial__tip">
              💡 {step.tip}
            </div>
          )}
        </div>

        {/* Controls */}
        {!step.hideControls && (
          <div className="ide-tutorial__controls">
            <button
              className="ide-tutorial__btn ide-tutorial__btn--skip"
              onClick={skipTutorial}
            >
              Skip Tutorial
            </button>

            <div className="ide-tutorial__nav">
              {currentStep > 0 && (
                <button
                  className="ide-tutorial__btn ide-tutorial__btn--prev"
                  onClick={prevStep}
                >
                  ← Previous
                </button>
              )}

              <button
                className="ide-tutorial__btn ide-tutorial__btn--next"
                onClick={nextStep}
              >
                {currentStep < steps.length - 1 ? (
                  <>Next →</>
                ) : (
                  step.buttonText || "Complete ✓"
                )}
              </button>
            </div>
          </div>
        )}

        {/* Keyboard hints */}
        <div className="ide-tutorial__hints">
          <span>Use arrow keys to navigate • ESC to skip</span>
        </div>
      </div>
    </>
  );
}

IDETutorial.propTypes = {
  steps: PropTypes.arrayOf(
    PropTypes.shape({
      title: PropTypes.string.isRequired,
      text: PropTypes.string.isRequired,
      target: PropTypes.string,
      position: PropTypes.oneOf(["top", "bottom", "left", "right", "center"]),
      offset: PropTypes.number,
      scrollIntoView: PropTypes.bool,
      backdropClickable: PropTypes.bool,
      hideControls: PropTypes.bool,
      buttonText: PropTypes.string,
      warning: PropTypes.string,
      tip: PropTypes.string,
    })
  ).isRequired,
  onComplete: PropTypes.func,
  onSkip: PropTypes.func,
  autoStart: PropTypes.bool,
};

export default IDETutorial;
