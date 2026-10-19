FROM python:3.12-slim

ENV PYTHONUNBUFFERED=1 PIP_NO_CACHE_DIR=1 DATA_DIR=/data
WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt
COPY app/ .

# Non-root user in the dialout group (gid 20, same as the host) for serial access.
RUN useradd --uid 1000 --gid 20 --no-create-home emio && mkdir -p /data && chown emio /data
USER emio

EXPOSE 8000
CMD ["uvicorn", "server:app", "--host", "0.0.0.0", "--port", "8000"]
