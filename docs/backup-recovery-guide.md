# Backup and Recovery Guide for Qdrant Vector Database

This document provides comprehensive procedures for backing up and recovering the Qdrant vector database used in the RAG Chatbot system.

## Table of Contents
- [Overview](#overview)
- [Backup Strategies](#backup-strategies)
- [Manual Backup Procedures](#manual-backup-procedures)
- [Automated Backup Solutions](#automated-backup-solutions)
- [Recovery Procedures](#recovery-procedures)
- [Disaster Recovery Planning](#disaster-recovery-planning)
- [Testing Backup and Recovery](#testing-backup-and-recovery)
- [Best Practices](#best-practices)

## Overview

The RAG Chatbot system uses Qdrant as its vector database to store and retrieve document embeddings. Proper backup and recovery procedures are essential to ensure data integrity and minimize downtime in case of failures.

### Database Structure
- **Collections**: Each book or document set may have its own collection
- **Vectors**: Embeddings of text chunks with metadata
- **Payloads**: Additional metadata associated with vectors
- **Indexes**: Vector similarity indexes for fast retrieval

## Backup Strategies

### 1. Full Backup
- Complete copy of all collections and data
- Recommended frequency: Weekly
- Storage: Cloud storage (AWS S3, Google Cloud Storage, etc.)

### 2. Incremental Backup
- Changes since last full backup
- Recommended frequency: Daily
- Storage: Cloud storage with versioning

### 3. Snapshot Backup
- Point-in-time snapshot of the database
- Recommended frequency: Hourly for production
- Storage: Qdrant's native snapshot feature

## Manual Backup Procedures

### Qdrant Snapshot Creation

1. **Create Collection Snapshot**:
   ```bash
   curl -X POST \
     'http://your-qdrant-url/collections/{collection_name}/snapshots' \
     -H 'Content-Type: application/json' \
     -d '{
       "snapshot_name": "backup_$(date +%Y%m%d_%H%M%S)"
     }'
   ```

2. **List Available Snapshots**:
   ```bash
   curl -X GET \
     'http://your-qdrant-url/collections/{collection_name}/snapshots'
   ```

3. **Download Snapshot**:
   ```bash
   curl -X GET \
     'http://your-qdrant-url/collections/{collection_name}/snapshots/{snapshot_name}' \
     -o /path/to/backup/{snapshot_name}.tar
   ```

### Using Qdrant CLI (if available)

```bash
# Create snapshot
qdrant-cli snapshot create --collection {collection_name} --name backup_$(date +%Y%m%d)

# List snapshots
qdrant-cli snapshot list --collection {collection_name}

# Download snapshot
qdrant-cli snapshot download --collection {collection_name} --snapshot {snapshot_name} --path /backup/location/
```

### Docker-based Backup (for self-hosted)

If using Docker, you can create backups by copying the data directory:

```bash
# Create backup of Qdrant data
docker cp qdrant_container:/qdrant/storage /backup/location/qdrant_backup_$(date +%Y%m%d)

# Compress backup
tar -czf qdrant_backup_$(date +%Y%m%d).tar.gz /backup/location/qdrant_backup_$(date +%Y%m%d)
```

## Automated Backup Solutions

### 1. Cron Job Script

Create a backup script that can be scheduled with cron:

```bash
#!/bin/bash
# qdrant_backup.sh

# Configuration
QDRANT_URL="http://your-qdrant-url"
COLLECTION_NAME="book_chunks"  # Replace with your collection name
BACKUP_DIR="/backup/qdrant"
DATE=$(date +%Y%m%d_%H%M%S)

# Create snapshot
SNAPSHOT_NAME="backup_$DATE"
curl -X POST \
  "$QDRANT_URL/collections/$COLLECTION_NAME/snapshots" \
  -H 'Content-Type: application/json' \
  -d "{\"snapshot_name\": \"$SNAPSHOT_NAME\"}"

# Wait for snapshot creation (adjust sleep time as needed)
sleep 30

# Download snapshot
curl -X GET \
  "$QDRANT_URL/collections/$COLLECTION_NAME/snapshots/$SNAPSHOT_NAME" \
  -o "$BACKUP_DIR/$SNAPSHOT_NAME.tar"

# Cleanup old snapshots (keep last 7 days)
find $BACKUP_DIR -name "*.tar" -mtime +7 -delete

# Upload to cloud storage (example with AWS S3)
aws s3 cp $BACKUP_DIR/$SNAPSHOT_NAME.tar s3://your-backup-bucket/qdrant/

echo "Backup completed: $SNAPSHOT_NAME"
```

### 2. Kubernetes CronJob (for K8s deployments)

```yaml
apiVersion: batch/v1
kind: CronJob
metadata:
  name: qdrant-backup
spec:
  schedule: "0 2 * * *"  # Daily at 2 AM
  jobTemplate:
    spec:
      template:
        spec:
          containers:
          - name: qdrant-backup
            image: curlimages/curl
            command:
            - /bin/sh
            - -c
            - |
              # Create snapshot
              SNAPSHOT_NAME="backup-$(date +%Y%m%d-%H%M%S)"
              curl -X POST \
                "http://qdrant-service:6333/collections/book_chunks/snapshots" \
                -H 'Content-Type: application/json' \
                -d "{\"snapshot_name\": \"$SNAPSHOT_NAME\"}"

              # Wait for creation
              sleep 30

              # Download snapshot
              curl -X GET \
                "http://qdrant-service:6333/collections/book_chunks/snapshots/$SNAPSHOT_NAME" \
                -o "/tmp/$SNAPSHOT_NAME.tar"

              # Upload to S3
              aws s3 cp "/tmp/$SNAPSHOT_NAME.tar" "s3://your-bucket/qdrant-backups/"
          restartPolicy: OnFailure
```

### 3. Cloud Provider Solutions

For Qdrant Cloud, utilize the built-in backup features:

1. **Qdrant Cloud Dashboard**: Configure automatic backups
2. **Cloud Storage Integration**: Set up automatic snapshot storage
3. **Cross-Region Replication**: Enable for disaster recovery

## Recovery Procedures

### 1. Collection Recovery from Snapshot

```bash
# Recover collection from snapshot
curl -X POST \
  'http://your-qdrant-url/collections/{collection_name}/snapshots/recover' \
  -H 'Content-Type: application/json' \
  -d '{
    "location": "http://your-storage/snapshots/snapshot_name.tar",
    "priority": "replica_first"
  }'
```

### 2. Complete Database Recovery

1. **Stop the application** to prevent write conflicts
2. **Drop existing collection** (if needed):
   ```bash
   curl -X DELETE \
     'http://your-qdrant-url/collections/{collection_name}'
   ```
3. **Create new collection** with same configuration:
   ```bash
   curl -X PUT \
     'http://your-qdrant-url/collections/{collection_name}' \
     -H 'Content-Type: application/json' \
     -d '{
       "vector_size": 768,
       "distance": "Cosine"
     }'
   ```
4. **Recover from snapshot**:
   ```bash
   curl -X POST \
     'http://your-qdrant-url/collections/{collection_name}/snapshots/recover' \
     -H 'Content-Type: application/json' \
     -d '{
       "location": "path/to/snapshot.tar"
     }'
   ```

### 3. Docker-based Recovery

```bash
# Stop Qdrant container
docker stop qdrant_container

# Restore data directory
rm -rf /path/to/qdrant/storage
tar -xzf /path/to/backup/qdrant_backup.tar.gz -C /path/to/qdrant/

# Start Qdrant container
docker start qdrant_container
```

## Disaster Recovery Planning

### Recovery Time Objective (RTO)
- **Critical**: < 1 hour for production systems
- **Standard**: < 4 hours for development systems

### Recovery Point Objective (RPO)
- **Critical**: < 1 hour data loss tolerance
- **Standard**: < 24 hours data loss tolerance

### Backup Retention Policy
- **Hourly snapshots**: Keep for 7 days
- **Daily snapshots**: Keep for 30 days
- **Weekly snapshots**: Keep for 6 months
- **Monthly snapshots**: Keep for 2 years

### Offsite Storage
- Store backups in geographically separate locations
- Use multiple cloud providers for redundancy
- Encrypt backups in transit and at rest

## Testing Backup and Recovery

### Regular Testing Schedule
- **Monthly**: Full recovery test
- **Weekly**: Backup verification
- **Daily**: Backup existence check

### Backup Verification Script

```bash
#!/bin/bash
# verify_backup.sh

BACKUP_DIR="/backup/qdrant"
MAX_AGE=2  # Hours

# Check if recent backup exists
RECENT_BACKUP=$(find $BACKUP_DIR -name "*.tar" -mmin -$((MAX_AGE * 60)) | head -n 1)

if [ -z "$RECENT_BACKUP" ]; then
    echo "ERROR: No recent backup found (last $MAX_AGE hours)"
    exit 1
else
    echo "OK: Recent backup found: $RECENT_BACKUP"

    # Check file size (basic integrity check)
    SIZE=$(stat -c%s "$RECENT_BACKUP")
    if [ $SIZE -lt 1024 ]; then  # Less than 1KB
        echo "ERROR: Backup file too small, may be corrupted"
        exit 1
    else
        echo "OK: Backup file size: $SIZE bytes"
    fi
fi

echo "Backup verification completed successfully"
```

### Recovery Test Procedure
1. **Create test environment** separate from production
2. **Restore backup** to test environment
3. **Verify data integrity** by running sample queries
4. **Document results** and update procedures as needed

## Best Practices

### Security
- **Encrypt backups** using AES-256 encryption
- **Secure access** with proper authentication and authorization
- **Audit access** to backup files and recovery operations
- **Use signed URLs** for temporary access to backups

### Performance
- **Schedule backups** during low-usage periods
- **Use incremental backups** to reduce storage and network usage
- **Monitor backup performance** and adjust schedules as needed
- **Compress backups** to reduce storage costs

### Monitoring
- **Alert on backup failures** immediately
- **Monitor backup size** for unusual changes
- **Track recovery times** to meet RTO requirements
- **Log all backup and recovery operations**

### Documentation
- **Maintain current procedures** and update regularly
- **Document collection schemas** for recovery
- **Keep contact information** for emergency recovery
- **Record backup and recovery times** for analysis

## Emergency Recovery Contacts

- **Database Administrator**: [Contact Information]
- **System Administrator**: [Contact Information]
- **Cloud Provider Support**: [Support Contact]
- **Development Team**: [On-call Contact]

## Conclusion

Regular backups and tested recovery procedures are critical for maintaining the availability and integrity of your Qdrant vector database. Follow these procedures to ensure your RAG Chatbot system remains resilient to data loss and can quickly recover from failures.