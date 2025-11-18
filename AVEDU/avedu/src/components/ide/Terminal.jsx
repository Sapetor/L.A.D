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

  // Execute command
  const executeCommand = useCallback(
    (command, term) => {
      // Built-in commands
      if (command === "help") {
        term.writeln("\x1b[1;36mAvailable commands:\x1b[0m");
        term.writeln("  \x1b[32mhelp\x1b[0m          - Show this help message");
        term.writeln("  \x1b[32mclear\x1b[0m         - Clear the terminal");
        term.writeln("  \x1b[32mls\x1b[0m            - List files");
        term.writeln("  \x1b[32mpwd\x1b[0m           - Print working directory");
        term.writeln("");
        term.writeln("\x1b[90mROS commands will be executed in Docker when backend is ready\x1b[0m");
      } else if (command === "clear") {
        term.clear();
      } else if (command === "pwd") {
        term.writeln(`${workingDirectory}/${username}/${canvasId}`);
      } else {
        // Call parent handler for actual execution
        if (onCommandExecute) {
          term.writeln(`\x1b[90mExecuting: ${command}\x1b[0m`);
          onCommandExecute(command, (output) => {
            term.writeln(output);
            writePrompt(term);
          });
          return; // Don't write prompt yet, wait for callback
        } else {
          term.writeln(`\x1b[33m⚠ Backend not connected\x1b[0m`);
          term.writeln(`\x1b[90mCommand: ${command}\x1b[0m`);
        }
      }

      writePrompt(term);
    },
    [onCommandExecute, workingDirectory, username, canvasId, writePrompt]
  );

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
