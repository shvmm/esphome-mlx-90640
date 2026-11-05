// Injected into the ESPHome web_server home page (web_server: js_include).
// Adds a link bar above the standard dashboard. Edit LINKS to taste;
// changes take effect after the next compile + flash.
const LINKS = [
  { label: "Heatmap", href: "/thermal", icon: "\u{1F321}️", primary: true },
  { label: "Snapshot (JPEG)", href: "/thermal.jpg", icon: "\u{1F4F7}" },
  { label: "Raw data (JSON)", href: "/thermal.json", icon: "{ }" },
  { label: "Home Assistant", href: "http://homeassistant.local:8123", icon: "\u{1F3E0}", external: true },
  { label: "ESPHome docs", href: "https://esphome.io", icon: "\u{1F4D6}", external: true },
  {
    label: "MLX90640 (Melexis)",
    href: "https://github.com/melexis/mlx90640-library",
    icon: "\u{1F4C4}",
    external: true,
  },
  {
    label: "XIAO ESP32-S3",
    href: "https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/",
    icon: "\u{1F4C4}",
    external: true,
  },
];

const css = `
#thermal-links{display:flex;flex-wrap:wrap;align-items:center;gap:8px;padding:10px 16px;
  font:14px system-ui,sans-serif;border-bottom:1px solid rgba(127,127,127,.25)}
#thermal-links a{display:inline-flex;align-items:center;gap:6px;padding:6px 12px;border-radius:16px;
  text-decoration:none;color:inherit;border:1px solid rgba(127,127,127,.35)}
#thermal-links a:hover{background:rgba(127,127,127,.15)}
#thermal-links a.primary{background:#e8590c;border-color:#e8590c;color:#fff;font-weight:600}
#thermal-links a.primary:hover{background:#d9480f}
#thermal-links .ext{opacity:.6;font-size:11px}
`;

function addLinkBar() {
  if (document.getElementById("thermal-links")) return;
  const style = document.createElement("style");
  style.textContent = css;
  document.head.appendChild(style);

  const nav = document.createElement("nav");
  nav.id = "thermal-links";
  for (const l of LINKS) {
    const a = document.createElement("a");
    a.href = l.href;
    if (l.primary) a.className = "primary";
    if (l.external) {
      a.target = "_blank";
      a.rel = "noopener";
    }
    a.textContent = `${l.icon} ${l.label}`;
    if (l.external) {
      const s = document.createElement("span");
      s.className = "ext";
      s.textContent = "↗";
      a.appendChild(s);
    }
    nav.appendChild(a);
  }
  document.body.prepend(nav);
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", addLinkBar);
} else {
  addLinkBar();
}
