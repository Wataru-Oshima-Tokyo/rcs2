from django.contrib import admin
from django.urls import path, include, re_path
from . import views

urlpatterns = [
    path("", views.index, name="index"),
    path("robot/<str:robot_name>/", views.robot, name="robot"),
    path("locale/<str:locale_name>/", views.locale, name="locale"),
]
