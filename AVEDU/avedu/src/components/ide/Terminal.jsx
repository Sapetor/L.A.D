// components/ide/Terminal.jsx
import React, { useEffect, useRef, useCallback } from "react";
import PropTypes from "prop-types";
import "./Terminal.scss";

// NOTE: This component uses @xterm packages
// Installed: npm install @xterm/xterm @xterm/addon-fit @xterm/addon-web-links

import { Terminal as XTerm } from "@xterm/xterm";
import { FitAddon } from "@xterm/addon-fit";
import { WebLinksAddon } from "@xterm/addon-web-links";
import "@xterm/xterm/css/xterm.css";

/**
 * Terminal component with xterm.js integration
 *
 * @param {Object} props
 * @param {Function} props.onCommandExecute - Callback when user executes a command
 * @param {string} props.workingDirectory - Current working directory
 * @param {string} props.username - Username to display in prompt
 * @param {string} props.canvasId - Canvas/workspace ID
 */
export function Terminal({
  onCommandExecute,
  workingDirectory = "/workspaces",
  username = "user",
  canvasId = "workspace",
}) {
  const terminalRef = useRef(null);
  const xtermRef = useRef(null);
  const fitAddonRef = useRef(null);
  const currentLineRef = useRef("");
  const historyRef = useRef([]);
  const historyIndexRef = useRef(-1);
  const runningProcessRef = useRef(null); // { processId, pollInterval }
  const isExecutingRef = useRef(false); // Prevent multiple commands at once

  // Initialize terminal
  useEffect(() => {
    if (!terminalRef.current) return;

    const term = new XTerm({
      cursorBlink: true,
      cursorStyle: "block",
      fontFamily: '"Cascadia Code", "Fira Code", Consolas, monospace',
      fontSize: 14,
      lineHeight: 1.4,
      theme: {
        background: "#0b1020",
        foreground: "#e6f1ff",
        cursor: "#7df9ff",
        cursorAccent: "#0b1020",
        selection: "rgba(125, 249, 255, 0.3)",
        black: "#1a1f2e",
        red: "#ff5c5c",
        green: "#7df9ff",
        yellow: "#ffcd1c",
        blue: "#5c9cff",
        magenta: "#ff5cf4",
        cyan: "#7df9ff",
        white: "#e6f1ff",
        brightBlack: "#475569",
        brightRed: "#ff8080",
        brightGreen: "#a0f7ff",
        brightYellow: "#ffd74d",
        brightBlue: "#80b3ff",
        brightMagenta: "#ff8cff",
        brightCyan: "#a0f7ff",
        brightWhite: "#ffffff",
      },
      allowTransparency: true,
      scrollback: 1000,
    });

    // Add fit addon for responsive sizing
    const fitAddon = new FitAddon();
    term.loadAddon(fitAddon);

    // Add web links addon
    const webLinksAddon = new WebLinksAddon();
    term.loadAddon(webLinksAddon);

    // Open terminal
    term.open(terminalRef.current);

    xtermRef.current = term;
    fitAddonRef.current = fitAddon;

    // Fit terminal after a short delay to ensure DOM is ready
    setTimeout(() => {
      try {
        fitAddon.fit();
      } catch (error) {
        console.warn("Failed to fit terminal on initial load:", error);
      }
    }, 100);

    // Welcome message
    term.writeln("\x1b[1;36m╔═══════════════════════════════════════════╗\x1b[0m");
    term.writeln("\x1b[1;36m║\x1b[0m  \x1b[1;35mROS Visual IDE Terminal\x1b[0m                \x1b[1;36m║\x1b[0m");
    term.writeln("\x1b[1;36m╚═══════════════════════════════════════════╝\x1b[0m");
    term.writeln("");
    term.writeln(`\x1b[90mWorking directory: ${workingDirectory}/${username}/${canvasId}\x1b[0m`);
    term.writeln(`\x1b[90mType 'help' for available commands\x1b[0m`);
    term.writeln("");
    writePrompt(term);

    // Handle terminal input
    term.onData((data) => {
      handleTerminalInput(data, term);
    });

    // Handle resize with error handling
    const resizeObserver = new ResizeObserver(() => {
      try {
        if (fitAddon && term) {
          fitAddon.fit();
        }
      } catch (error) {
        // Silently ignore resize errors during cleanup
        console.debug("Terminal resize error:", error);
      }
    });
    resizeObserver.observe(terminalRef.current);

    // Cleanup
    return () => {
      resizeObserver.disconnect();
      term.dispose();
    };
  }, [workingDirectory, username, canvasId]);

  // Write prompt
  const writePrompt = useCallback((term) => {
    term.write(`\r\n\x1b[1;32m${username}@ros\x1b[0m:\x1b[1;34m~/${canvasId}\x1b[0m$ `);
  }, [username, canvasId]);

  // Handle terminal input
  const handleTerminalInput = useCallback(
    (data, term) => {
      const code = data.charCodeAt(0);

      // Ctrl+C - Kill running process
      if (code === 3) {
        if (runningProcessRef.current) {
          term.writeln("\r\n\x1b[33m^C\x1b[0m");
          term.writeln("\x1b[90mKilling process...\x1b[0m");
          killRunningProcess(term);
        } else {
          term.writeln("\r\n^C");
          currentLineRef.current = "";
          writePrompt(term);
        }
        return;
      }

      // Block input if a command is already executing
      if (isExecutingRef.current) {
        return;
      }

      // Enter key
      if (code === 13) {
        const command = currentLineRef.current.trim();
        term.write("\r\n");

        if (command) {
          // Add to history
          historyRef.current.push(command);
          historyIndexRef.current = historyRef.current.length;

          // Execute command
          executeCommand(command, term);
        } else {
          writePrompt(term);
        }

        currentLineRef.current = "";
      }
      // Backspace
      else if (code === 127 || code === 8) {
        if (currentLineRef.current.length > 0) {
          currentLineRef.current = currentLineRef.current.slice(0, -1);
          term.write("\b \b");
        }
      }
      // Arrow up (history previous)
      else if (data === "\x1b[A") {
        if (historyIndexRef.current > 0) {
          historyIndexRef.current--;
          const historyCommand = historyRef.current[historyIndexRef.current];
          clearCurrentLine(term, currentLineRef.current.length);
          currentLineRef.current = historyCommand;
          term.write(historyCommand);
        }
      }
      // Arrow down (history next)
      else if (data === "\x1b[B") {
        if (historyIndexRef.current < historyRef.current.length - 1) {
          historyIndexRef.current++;
          const historyCommand = historyRef.current[historyIndexRef.current];
          clearCurrentLine(term, currentLineRef.current.length);
          currentLineRef.current = historyCommand;
          term.write(historyCommand);
        } else {
          historyIndexRef.current = historyRef.current.length;
          clearCurrentLine(term, currentLineRef.current.length);
          currentLineRef.current = "";
        }
      }
      // Tab key - autocomplete (basic implementation)
      else if (code === 9) {
        // For now, just prevent default tab behavior
        // TODO: Implement actual autocomplete with backend support
        const currentInput = currentLineRef.current;

        // Basic autocomplete for common commands
        const commonCommands = [
          'source', 'ls', 'cd', 'pwd', 'mkdir', 'rm', 'cat', 'echo',
          'ros2', 'colcon', 'python3', 'pip', 'git',
          'ros2 run', 'ros2 topic list', 'ros2 topic echo',
          'ros2 node list', 'ros2 pkg create',
          'colcon build', 'source install/setup.bash', 'source install/local_setup.bash'
        ];

        const matches = commonCommands.filter(cmd => cmd.startsWith(currentInput));

        if (matches.length === 1) {
          // Single match - autocomplete
          const completion = matches[0].slice(currentInput.length);
          currentLineRef.current += completion;
          term.write(completion);
        } else if (matches.length > 1) {
          // Multiple matches - show options
          term.write('\r\n');
          matches.forEach(match => {
            term.writeln(`  ${match}`);
          });
          writePrompt(term);
          term.write(currentInput);
        }
      }
      // Regular character
      else if (code >= 32) {
        currentLineRef.current += data;
        term.write(data);
      }
    },
    [writePrompt]
  );

  // Clear current line
  const clearCurrentLine = (term, length) => {
    for (let i = 0; i < length; i++) {
      term.write("\b \b");
    }
  };

  // Helper: Check if command is long-running (needs streaming)
  const isLongRunningCommand = (command) => {
    const streamingPatterns = [
      /^ros2\s+run\s+/,          // ros2 run
      /^ros2\s+topic\s+echo\s+/, // ros2 topic echo
      /^ros2\s+bag\s+play\s+/,   // ros2 bag play
      /^python3?\s+.*\.py/,      // Python scripts (might run forever)
      /^colcon\s+build/,         // colcon build
      /^gazebo/,                 // Gazebo
      /^rviz2?/,                 // RViz
      /tail\s+-f/,               // tail -f
    ];

    return streamingPatterns.some(pattern => pattern.test(command));
  };

  // Kill running process
  const killRunningProcess = useCallback(async (term) => {
    if (!runningProcessRef.current) return;

    const { processId, pollInterval } = runningProcessRef.current;

    // Stop polling
    if (pollInterval) {
      clearInterval(pollInterval);
    }

    try {
      // Import fileApi dynamically
      const fileApi = await import('../../services/fileApi');
      await fileApi.killProcess(canvasId, processId);

      term.writeln(`\x1b[33m✓ Process killed\x1b[0m`);
    } catch (error) {
      term.writeln(`\x1b[31m✗ Failed to kill process: ${error.message}\x1b[0m`);
    } finally {
      runningProcessRef.current = null;
      isExecutingRef.current = false;
      writePrompt(term);
    }
  }, [canvasId, writePrompt]);

  // Poll for process output
  const pollProcessOutput = useCallback(async (processId, term) => {
    try {
      const fileApi = await import('../../services/fileApi');
      const result = await fileApi.getProcessOutput(canvasId, processId);

      // Write new output lines
      if (result.output && result.output.length > 0) {
        result.output.forEach(({ type, data }) => {
          const color = type === 'stderr' ? '\x1b[31m' : ''; // Red for stderr
          term.writeln(`${color}${data}\x1b[0m`);
        });
      }

      // Check if process completed
      if (result.status === 'completed') {
        // Stop polling
        if (runningProcessRef.current?.pollInterval) {
          clearInterval(runningProcessRef.current.pollInterval);
        }
        runningProcessRef.current = null;
        isExecutingRef.current = false;

        // Show completion message
        if (result.exit_code === 0) {
          term.writeln(`\x1b[32m✓ Process completed successfully\x1b[0m`);
        } else {
          term.writeln(`\x1b[31m✗ Process exited with code ${result.exit_code}\x1b[0m`);
        }

        writePrompt(term);
      }
    } catch (error) {
      // If process not found, stop polling
      if (runningProcessRef.current?.pollInterval) {
        clearInterval(runningProcessRef.current.pollInterval);
      }
      runningProcessRef.current = null;
      isExecutingRef.current = false;

      term.writeln(`\x1b[31m✗ Error: ${error.message}\x1b[0m`);
      writePrompt(term);
    }
  }, [canvasId, writePrompt]);

  // Execute command (streaming or normal)
  const executeCommand = useCallback(
    async (command, term) => {
      // Built-in commands
      if (command === "help") {
        term.writeln("\x1b[1;36m╔══════════════════════════════════════════╗\x1b[0m");
        term.writeln("\x1b[1;36m║\x1b[0m  \x1b[1;35mROS2 Workspace Commands\x1b[0m                \x1b[1;36m║\x1b[0m");
        term.writeln("\x1b[1;36m╚══════════════════════════════════════════╝\x1b[0m");
        term.writeln("");
        term.writeln("\x1b[1;33mBuilt-in Commands:\x1b[0m");
        term.writeln("  \x1b[32mhelp\x1b[0m          - Show this help message");
        term.writeln("  \x1b[32mclear\x1b[0m         - Clear the terminal");
        term.writeln("  \x1b[32mls\x1b[0m            - List files and directories");
        term.writeln("  \x1b[32mpwd\x1b[0m           - Print working directory");
        term.writeln("");
        term.writeln("\x1b[1;33mCommon ROS2 Commands:\x1b[0m");
        term.writeln("  \x1b[36mros2 topic list\x1b[0m                - List all topics");
        term.writeln("  \x1b[36mros2 topic echo /topic\x1b[0m         - Listen to topic");
        term.writeln("  \x1b[36mros2 topic hz /topic\x1b[0m           - Show topic rate");
        term.writeln("  \x1b[36mros2 node list\x1b[0m                 - List all nodes");
        term.writeln("  \x1b[36mros2 pkg create\x1b[0m                - Create package");
        term.writeln("");
        term.writeln("\x1b[1;33mPython Commands:\x1b[0m");
        term.writeln("  \x1b[36mpython3 script.py\x1b[0m              - Run Python script");
        term.writeln("  \x1b[36mcolcon build\x1b[0m                   - Build workspace");
        term.writeln("");
        term.writeln("\x1b[90mAll commands are executed in the Docker container\x1b[0m");
        term.writeln("");
        term.writeln("\x1b[33mTip: Press Ctrl+C to stop running processes\x1b[0m");
        writePrompt(term);
        return;
      }

      if (command === "clear") {
        term.clear();
        writePrompt(term);
        return;
      }

      if (command === "pwd") {
        term.writeln(`\x1b[36m${workingDirectory}\x1b[0m`);
        writePrompt(term);
        return;
      }

      // Check if this is a long-running command
      if (isLongRunningCommand(command)) {
        // Use streaming execution
        term.writeln(`\x1b[90m❯ Starting: ${command}\x1b[0m`);
        term.writeln(`\x1b[90m(Press Ctrl+C to stop)\x1b[0m`);
        isExecutingRef.current = true;

        try {
          const fileApi = await import('../../services/fileApi');
          const result = await fileApi.startStreamingCommand(canvasId, command);

          if (result.process_id) {
            // Start polling for output
            const pollInterval = setInterval(() => {
              pollProcessOutput(result.process_id, term);
            }, 500); // Poll every 500ms

            runningProcessRef.current = {
              processId: result.process_id,
              pollInterval,
            };

            // Do initial poll
            pollProcessOutput(result.process_id, term);
          }
        } catch (error) {
          isExecutingRef.current = false;
          term.writeln(`\x1b[31m✗ Failed to start command: ${error.message}\x1b[0m`);
          writePrompt(term);
        }
      } else {
        // Use normal execution for quick commands
        if (onCommandExecute) {
          term.writeln(`\x1b[90m❯ Executing: ${command}\x1b[0m`);
          isExecutingRef.current = true;

          onCommandExecute(command, (output) => {
            isExecutingRef.current = false;

            // Handle different output types
            if (output && typeof output === "string") {
              const trimmedOutput = output.trim();

              // If output is empty but command succeeded, show success message
              if (!trimmedOutput || trimmedOutput.length === 0) {
                // Check if it's a source command
                if (command.startsWith('source')) {
                  term.writeln(`\x1b[32m✓ Environment sourced successfully\x1b[0m`);
                } else {
                  term.writeln(`\x1b[32m✓ Command completed successfully\x1b[0m`);
                }
              } else {
                // Split output by lines and colorize based on content
                const lines = trimmedOutput.split("\n");
                lines.forEach((line) => {
                  if (line.includes("error") || line.includes("Error") || line.includes("ERROR")) {
                    term.writeln(`\x1b[31m${line}\x1b[0m`); // Red for errors
                  } else if (line.includes("warning") || line.includes("Warning") || line.includes("WARN")) {
                    term.writeln(`\x1b[33m${line}\x1b[0m`); // Yellow for warnings
                  } else if (line.includes("success") || line.includes("Success") || line.includes("✓")) {
                    term.writeln(`\x1b[32m${line}\x1b[0m`); // Green for success
                  } else {
                    term.writeln(line);
                  }
                });
              }
            } else {
              term.writeln(String(output));
            }
            writePrompt(term);
          });
        } else {
          term.writeln(`\x1b[33m⚠ Backend not connected\x1b[0m`);
          term.writeln(`\x1b[90mCommand: ${command}\x1b[0m`);
          writePrompt(term);
        }
      }
    },
    [onCommandExecute, canvasId, workingDirectory, writePrompt, pollProcessOutput, isLongRunningCommand]
  );

  // Listen for executeCommand events from ConvertToCodeNode
  useEffect(() => {
    const handleExecuteCommandEvent = (event) => {
      const { command } = event.detail;
      const term = xtermRef.current;

      if (command && term) {
        // Write the command to terminal
        currentLineRef.current = command;
        term.write(command);
        term.write("\r\n");

        // Execute it
        executeCommand(command, term);
        currentLineRef.current = "";
      }
    };

    window.addEventListener('executeCommand', handleExecuteCommandEvent);

    return () => {
      window.removeEventListener('executeCommand', handleExecuteCommandEvent);
    };
  }, [executeCommand]);

  // Public method to write output to terminal
  const writeOutput = useCallback((text, type = "info") => {
    if (!xtermRef.current) return;

    const colors = {
      info: "\x1b[0m",
      success: "\x1b[32m",
      error: "\x1b[31m",
      warning: "\x1b[33m",
    };

    xtermRef.current.writeln(`${colors[type]}${text}\x1b[0m`);
  }, []);

  return <div ref={terminalRef} className="terminal" />;
}

Terminal.propTypes = {
  onCommandExecute: PropTypes.func,
  workingDirectory: PropTypes.string,
  username: PropTypes.string,
  canvasId: PropTypes.string,
};

export default Terminal;
