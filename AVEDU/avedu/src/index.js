// src/index.jsx
import React from "react";
import { createRoot } from "react-dom/client";
import { BrowserRouter } from "react-router-dom";
import App from "./App";
import AuthProvider from "./context/AuthContext";
import { IP } from "./ip";

/**
 * Host detection (auto-configured by start-all script via REACT_APP_HOST)
 * Falls back to localhost if not set
 */
const HOST = process.env.REACT_APP_HOST || "localhost";

/** (Opcional) si cambian puertos en ese PC:
 * const PORTS = { ROSBRIDGE: 9090, STATIC: 7000, API: 8000, WVS: 8080 };
 */

createRoot(document.getElementById("root")).render(
  <React.StrictMode>
    <IP host={HOST /* , ports: PORTS */}>
      <BrowserRouter>
        <AuthProvider>
          <App />
        </AuthProvider>
      </BrowserRouter>
    </IP>
  </React.StrictMode>
);
