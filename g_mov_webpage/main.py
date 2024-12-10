# Importa la función create_app desde el módulo website.
# Esta función es responsable de crear y configurar la instancia de la aplicación Flask.
from website import create_app

# Llama a la función create_app para crear una instancia de la aplicación.
# Esta instancia es la que se usará para manejar las solicitudes HTTP.
app = create_app()

# Este bloque verifica si el script se está ejecutando directamente.
# Si es así, se ejecuta el código dentro de este bloque.
if __name__ == '__main__':
    # Inicia el servidor de desarrollo de Flask.
    # La opción debug=True activa el modo de depuración, lo que permite:
    # - Recargar automáticamente la aplicación cuando se detectan cambios en el código.
    # - Mostrar un rastreo de errores más detallado en el navegador en caso de que ocurran excepciones.
    app.run(debug=True)

