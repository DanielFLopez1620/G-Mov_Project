# Importar las bibliotecas necesarias de Flask y otras dependencias
from flask import Flask, redirect, url_for
from flask_sqlalchemy import SQLAlchemy  # Para manejar la base de datos
from os import path  # Para manejar rutas de archivos
from flask_login import LoginManager, current_user  # Para manejar la autenticación de usuarios
from datetime import timedelta

# Instancia de SQLAlchemy para interactuar con la base de datos
db = SQLAlchemy()
DB_NAME = "database.db"  # Nombre del archivo de la base de datos

def create_app():
    # Función para crear y configurar la aplicación Flask
    app = Flask(__name__)  # Crear una instancia de la aplicación Flask
    app.config['SECRET_KEY'] = '12345'  # Clave secreta para proteger sesiones
    app.config['SQLALCHEMY_DATABASE_URI'] = f'sqlite:///{DB_NAME}'  # Configurar la URI de la base de datos
    db.init_app(app)  # Inicializar la base de datos con la aplicación

    # Configurar las opciones de cookies para "remember me"
    app.config['REMEMBER_COOKIE_DURATION'] = timedelta(days=30)  # Duración de la cookie "remember me"
    app.config['REMEMBER_COOKIE_SECURE'] = True  # Usar HTTPS
    app.config['REMEMBER_COOKIE_HTTPONLY'] = True  # Evitar acceso a la cookie desde JavaScript

    # Importar los módulos de vistas y autenticación.
    from .views import views
    from .auth import auth

    # Registrar los blueprints para manejar las rutas de vistas y autenticación
    app.register_blueprint(views, url_prefix='/')  # Registrar el blueprint de vistas
    app.register_blueprint(auth, url_prefix='/')  # Registrar el blueprint de autenticación

    # Importar los modelos User y Note
    from .models import User, Note
    
    # Crear las tablas de la base de datos si no existen
    with app.app_context():
        db.create_all()  # Crear todas las tablas definidas en los modelos

    # Configurar el manejo de inicio de sesión
    login_manager = LoginManager()  # Crear una instancia de LoginManager
    login_manager.login_view = 'auth.login'  # Definir la vista de inicio de sesión
    login_manager.init_app(app)  # Inicializar el LoginManager con la aplicación

    # Cargar el usuario desde la base de datos usando su ID
    @login_manager.user_loader
    def load_user(id):
        return User.query.get(int(id))  # Consultar el usuario por su ID

    return app  # Retornar la aplicación configurada

def create_database(app):
    if not path.exists('website/' + DB_NAME):
        db.create_all(app=app)
        print('Created Database!')
