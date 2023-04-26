from django.contrib import admin
from django.urls import path, include, re_path
from . import views

urlpatterns = [
    path("", views.index, name="index"),
    path("<str:template_name>/", views.generic, name="template"),
]
