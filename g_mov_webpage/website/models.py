# Importar la instancia de la base de datos
from . import db
# Importar UserMixin de Flask-Login para manejar la autenticación de usuarios
from flask_login import UserMixin
# Importar la función 'func' de SQLAlchemy para manejar funciones de base de datos
from sqlalchemy.sql import func

# Definición del modelo Note
class Note(db.Model):
    # Definir la columna 'id' como un entero que es la clave primaria
    id = db.Column(db.Integer, primary_key=True)
    # Definir la columna 'data' como un string con un límite de 10000 caracteres
    data = db.Column(db.String(10000))
    # Definir la columna 'date' como un tipo de dato DateTime, con zona horaria y valor por defecto como la fecha y hora actuales
    date = db.Column(db.DateTime(timezone=True), default=func.now())
    # Definir la columna 'user_id' como un entero que es una clave foránea referenciando la tabla 'user'
    user_id = db.Column(db.Integer, db.ForeignKey('user.id'))

# Definición del modelo User
class User(db.Model, UserMixin):
    # Definir la columna 'id' como un entero que es la clave primaria
    id = db.Column(db.Integer, primary_key=True)
    # Definir la columna 'email' como un string único con un límite de 150 caracteres
    email = db.Column(db.String(150), unique=True)
    # Definir la columna 'password' como un string con un límite de 150 caracteres
    password = db.Column(db.String(150))
    # Definir la columna 'first_name' como un string con un límite de 150 caracteres
    first_name = db.Column(db.String(150))
    #Definir la columna "residence" como un string con un limite de 150 caracteres
    residence = db.Column(db.String(150))
    #Definir la columna "elderAge" como un entero
    elderAge = db.Column(db.Integer)
    #Definir la columna "elderHeight" como un string con un limite de 150 caracteres
    elderHeight = db.Column(db.String(150))
    #Definir la columna "elderName" como un string con un limite de 150 caracteres
    elderName = db.Column(db.String(150))
    # Definir una relación con el modelo 'Note', permitiendo acceder a las notas de un usuario
    notes = db.relationship('Note')
