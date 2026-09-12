# Explorador técnico · ABB IRB 120

Web interactiva en español con modelo 3D, selección y aislamiento de conjuntos,
despiece ilustrativo, información de las seis articulaciones y componentes de una celda.

## Abrir la web

Con Python instalado, desde la raíz del repositorio:

```sh
python -m http.server 8000
```

Abre <http://localhost:8000/web/> en el navegador. En Windows también puedes usar
`py -m http.server 8000`. Para detener el servidor, pulsa Ctrl+C.

La web usa módulos JavaScript y carga un archivo glTF, por lo que debe abrirse mediante
un servidor HTTP. No requiere npm ni compilación; las dependencias están en `vendor/`.

## Archivos y créditos

- `index.html`, `style.css`, `app.js` y `data.js`: interfaz y comportamiento.
- `assets/scene.gltf` y `assets/scene.bin`: modelo 3D.
- Modelo «ABB IRB 120 Robot» de bluoppVR, bajo CC BY 4.0; fuente y atribución
  completas en [assets/license.txt](assets/license.txt).
- Three.js y sus utilidades: licencia MIT incluida en [vendor/LICENSE](vendor/LICENSE).

El despiece es ilustrativo; no representa una simulación del movimiento articular.
